// External midline continuation, authored before live skin transport so the
// fixed pelvic seam does not pulse in response to moving pouch compression.
#include "continuous_raphe_data.h"
static void ExtendSurfaceRaphe(unsigned char* controlled,UINT graftFirstVertex){
  static V3 normals[graftNormalGroupCount],displacement[graftCount];
  static V3 field[graftCount+collarFairGroupCount],directions[graftCount+collarFairGroupCount];
  memset(normals,0,sizeof(normals));
  for(UINT k=0;k<graftTriangleIndexCount;k+=3){
    UINT a=graftTriangleIndices[k],b=graftTriangleIndices[k+1],c=graftTriangleIndices[k+2];
    V3 n=Cross(graftDeformedPositions[c]-graftDeformedPositions[a],graftDeformedPositions[b]-graftDeformedPositions[a]);
    normals[graftNormalGroup[a]]=normals[graftNormalGroup[a]]+n;
    normals[graftNormalGroup[b]]=normals[graftNormalGroup[b]]+n;
    normals[graftNormalGroup[c]]=normals[graftNormalGroup[c]]+n;
  }
  RebuildUnifiedCollarNormals(controlled,graftFirstVertex);
  for(UINT i=0;i<graftCount;i++)field[i]=Unit(normals[graftNormalGroup[i]]);
  for(UINT g=0;g<collarFairGroupCount;g++)field[graftCount+g]=collarNormalSums[g];
  // Sample a smooth spatial normal field, rather than independent face fans.
  // This gives near-coincident seam vertices the same displacement direction.
  for(UINT i=0;i<graftCount+collarFairGroupCount;i++){
    if((i<graftCount?rapheGraft[i]:rapheCollar[i-graftCount])<1e-7f){directions[i]={};continue;}
    V3 n{};for(UINT k=rapheFieldRows[i];k<rapheFieldRows[i+1];k++)n=n+field[rapheFieldIDs[k]]*rapheFieldWeights[k];
    directions[i]=Unit(n);
  }
  float scale=max(.6f,min(1.8f,sqrtf(sliderValues[0]/sliderSpecs[0].def)));
  for(UINT i=0;i<graftCount;i++){
    UINT g=graftCollarGroups[i];
    displacement[i]=g!=65535u?directions[graftCount+g]*(scale*rapheCollar[g]):directions[i]*(scale*rapheGraft[i]);
  }
  float fraction=1.f;
  for(UINT k=0;k<graftTriangleIndexCount;k+=3){
    UINT a=graftTriangleIndices[k],b=graftTriangleIndices[k+1],c=graftTriangleIndices[k+2];
    fraction=min(fraction,SurfaceCorrectionLimit(graftDeformedPositions[a],graftDeformedPositions[b],graftDeformedPositions[c],displacement[a],displacement[b],displacement[c],.25f));
  }
  for(UINT t=0;t<collarNormalTriangleCount;t++){
    V3 p[3],d[3];
    for(UINT c=0;c<3;c++){
      UINT k=t*3+c,id=collarNormalTriangleIndices[k],g=collarNormalTriangleGroups[k];
      p[c]=ReadAnyCollarPosition(id,k,controlled,graftFirstVertex);
      d[c]=g!=65535u?directions[graftCount+g]*(scale*rapheCollar[g]):V3{};
    }
    fraction=min(fraction,SurfaceCorrectionLimit(p[0],p[1],p[2],d[0],d[1],d[2],.25f));
  }
  debugRapheFraction=fraction;
  for(UINT i=0;i<graftCount;i++)graftDeformedPositions[i]=graftDeformedPositions[i]+displacement[i]*fraction;
  for(UINT g=0;g<collarFairGroupCount;g++){
    if(rapheCollar[g]<1e-7f)continue;
    V3 shift=directions[graftCount+g]*(scale*rapheCollar[g]*fraction);
    for(UINT k=collarFairMemberOffsets[g];k<collarFairMemberOffsets[g+1];k++){
      UINT id=collarFairMembers[k];if(id>=graftFirstVertex)continue;
      unsigned char* v=controlled+(id-pelvisControlFirstVertex)*graftStride;
      V3 p;memcpy(&p,v,12);p=p+shift;memcpy(v,&p,12);
    }
  }
}
