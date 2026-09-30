#pragma once
// Final shared render skin. The established cage/pouch/tube still owns motion;
// this locally rebuilt junction consumes its final surface, including body fields.
static V3 nrPositions[nrCount],nrNormals[nrCount],nrTangents[nrCount];
static void UpdateNeckRender(){PerfScope perf(13);
 static float unpack[256];static bool unpackReady=false;
 if(!unpackReady){for(unsigned i=0;i<256;i++)unpack[i]=i/255.f*2-1;unpackReady=true;}
 static bool attributesReady=false;
 if(!attributesReady){
  for(unsigned i=0;i<nrCount;i++)memcpy(nrPacked+i*32+12,nrAttributes+i*20,20);
  attributesReady=true;
 }
 static V3 source[rsCount];
 static __m128 sourceXYZ[rsCount];
 for(unsigned i=0;i<rsCount;i++){
  memcpy(&source[i],rsPacked+i*32,12);
  sourceXYZ[i]=_mm_set_ps(0.f,source[i].z,source[i].y,source[i].x);
 }
 GeometryFor(nrCount,[&](unsigned i){
  if(nrDirect[i]!=65535){memcpy(nrPacked+i*32,rsPacked+nrDirect[i]*32,32);nrPositions[i]=source[nrDirect[i]];
 return;}
  // Evaluate XYZ together, retaining the exact weight accumulation order.
  __m128 sum=_mm_setzero_ps();
  for(unsigned j=nrRows[i];j<nrRows[i+1];j++)sum=_mm_add_ps(sum,_mm_mul_ps(sourceXYZ[nrSources[j]],_mm_set1_ps(nrWeights[j])));
  float xyz[4];_mm_storeu_ps(xyz,sum);V3 p{xyz[0],xyz[1],xyz[2]};
  nrPositions[i]=p;memcpy(nrPacked+i*32,&p,12);
 });
 static float uv[nrIndexCount/3][5];static bool uvReady=false;static std::vector<unsigned> changedFaces;
 if(!uvReady){for(unsigned k=0;k<nrIndexCount;k+=3){
  float tex[3][2];for(unsigned j=0;j<3;j++)D3DXFloat16To32Array(tex[j],reinterpret_cast<const D3DXFLOAT16*>(nrPacked+nrIndices[k+j]*32+28),2);
  float* d=uv[k/3];d[0]=tex[1][0]-tex[0][0];d[1]=tex[1][1]-tex[0][1];d[2]=tex[2][0]-tex[0][0];d[3]=tex[2][1]-tex[0][1];d[4]=d[0]*d[3]-d[1]*d[2];
  if(nrDirect[nrIndices[k]]==65535||nrDirect[nrIndices[k+1]]==65535||nrDirect[nrIndices[k+2]]==65535)changedFaces.push_back(k);
 }uvReady=true;}
 memset(nrNormals,0,sizeof(nrNormals));memset(nrTangents,0,sizeof(nrTangents));
 // Retained vertices copy their complete lighting basis from rsPacked.
 // Only faces incident to inserted vertices contribute to a basis we write.
 // Keep original triangle order so their accumulated floats remain exact.
 for(unsigned k:changedFaces){
  unsigned a=nrIndices[k],b=nrIndices[k+1],c=nrIndices[k+2];V3 e=nrPositions[b]-nrPositions[a],g=nrPositions[c]-nrPositions[a],n=Cross(g,e);
  nrNormals[a]=nrNormals[a]+n;nrNormals[b]=nrNormals[b]+n;nrNormals[c]=nrNormals[c]+n;
  const float* d=uv[k/3];if(fabsf(d[4])>1e-10f){V3 t=(e*d[3]-g*d[1])/d[4];nrTangents[a]=nrTangents[a]+t;nrTangents[b]=nrTangents[b]+t;nrTangents[c]=nrTangents[c]+t;}
 }
 GeometryFor(nrCount,[&](unsigned i){
  if(nrDirect[i]!=65535)return;
  V3 originalN{},originalT{};
  for(unsigned j=0;j<3;j++){const unsigned char* v=rsPacked+nrMaterialSources[i*3+j]*32;float w=nrMaterialWeights[i*3+j];
   originalN=originalN+V3{unpack[v[16]],unpack[v[17]],unpack[v[18]]}*w;
   originalT=originalT+V3{unpack[v[12]],unpack[v[13]],unpack[v[14]]}*w;
  }
  float blend=nrNormalMix[i];V3 n=Unit(Unit(originalN)*(1-blend)+Unit(nrNormals[i])*blend);
  V3 t=Unit(originalT)*(1-blend)+Unit(nrTangents[i])*blend;t=t-n*Dot(n,t);
  if(Length(t)<1e-6f)t=Cross(fabsf(n.z)<.9f?V3{0,0,1}:V3{0,1,0},n);t=Unit(t);
  unsigned char* v=nrPacked+i*32;v[12]=PackSigned(t.x);v[13]=PackSigned(t.y);v[14]=PackSigned(t.z);v[16]=PackSigned(n.x);v[17]=PackSigned(n.y);v[18]=PackSigned(n.z);
 });
}
