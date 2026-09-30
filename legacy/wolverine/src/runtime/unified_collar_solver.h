#pragma once
#include "unified_collar_data.h"
// One coupled energy owns the final collar and its recruited pelvic annulus.
// Fine boundary vertices are eliminated into their original body-edge ends.
// Keep this solver independent of the motion solver and packed skin weights.
#include <Eigen/SparseCholesky>
#include <Eigen/Geometry>
namespace UnifiedCollar {
using Sp=Eigen::SparseMatrix<double>;using Triplet=Eigen::Triplet<double>;using Mat=Eigen::Matrix<double,Eigen::Dynamic,3,Eigen::RowMajor>;
static Sp P,A,fixedCoupling;
static Eigen::SimplicialLDLT<Sp> factor;
static Eigen::VectorXd inverseDiagonal;
static std::vector<unsigned> masters,freeIDs,fixedIDs;
static std::vector<unsigned> normalFaces,packedIDs;
static std::vector<unsigned> targetIDs,projectionOffsets,projectionSources,boundaryOffsets,boundarySources;
static std::vector<double> projectionWeights,boundaryWeights;
static std::vector<int> masterOf,freeOf,fixedOf;
static std::vector<double> mask,screen;
static Mat before,target,solved;
static Mat transitionCorrection;
static unsigned transitionFrames=0;
 static unsigned revision=~0u;static int mode=-1,metricSequencePhase=0;static bool ready=false,valid=false;
static float cachedControls[9]{},cachedRadius=0,cachedLength=0;
static double buildMS=0,solveMS=0;static unsigned builds=0;
static double profile[5]{};
static unsigned char* Packed(unsigned i,unsigned char* body){
 if(i<nrCount)return nrPacked+i*32;
 i-=nrCount;
 if(i<sharedBodyCount[0])return body+(17449+i)*32;
 return body+(41435+i-sharedBodyCount[0])*32;
}
static V3 Point(const Mat& p,unsigned i){return {(float)p(i,0),(float)p(i,1),(float)p(i,2)};}
static void Topology(){
 if(ready)return;masterOf.assign(ucCount,-1);std::vector<bool> slave(ucCount,false);
 for(unsigned k=0;k<ucSeamCount;k++)slave[ucSeamVertices[k*3]]=true;
 for(unsigned i=0;i<ucCount;i++)if(!slave[i]){masterOf[i]=(int)masters.size();masters.push_back(i);}
 std::vector<Triplet> entries;entries.reserve(ucCount+ucSeamCount);
 for(unsigned j=0;j<masters.size();j++)entries.emplace_back(masters[j],j,1.);
 for(unsigned k=0;k<ucSeamCount;k++){unsigned x=ucSeamVertices[k*3],a=ucSeamVertices[k*3+1],b=ucSeamVertices[k*3+2];double u=ucSeamWeights[k];entries.emplace_back(x,masterOf[a],1-u);entries.emplace_back(x,masterOf[b],u);}
 P.resize(ucCount,(int)masters.size());P.setFromTriplets(entries.begin(),entries.end());ready=true;
}
static void Read(unsigned char* body){
 before.resize(ucCount,3);
 for(unsigned i=0;i<ucCount;i++){V3 p;memcpy(&p,Packed(ucKeep[i],body),12);before.row(i)<<p.x,p.y,p.z;}
 for(unsigned k=0;k<ucSeamCount;k++){unsigned x=ucSeamVertices[k*3],a=ucSeamVertices[k*3+1],b=ucSeamVertices[k*3+2];double u=ucSeamWeights[k];before.row(x)=before.row(a)*(1-u)+before.row(b)*u;}
}
static void Build(V3 root,V3 axis,V3 up,float radius,float length){
 double begin=PerfClock();std::vector<double> area(ucCount,0);std::vector<Triplet> c;c.reserve(ucFaceCount*12);
 for(unsigned k=0;k<ucFaceCount;k++){
  unsigned ids[3]={ucFaces[k*3],ucFaces[k*3+1],ucFaces[k*3+2]};
  Eigen::Vector3d p[3]={before.row(ids[0]),before.row(ids[1]),before.row(ids[2])};
  double twice=(p[1]-p[0]).cross(p[2]-p[0]).norm();if(twice<1e-14)continue;
  for(unsigned j=0;j<3;j++){
   unsigned a=ids[(j+1)%3],b=ids[(j+2)%3];double w=(p[(j+1)%3]-p[j]).dot(p[(j+2)%3]-p[j])/(2*twice);
   // Imported remesh slivers must not introduce negative diffusion or
   // unbounded stiffness into the metric at a collapsed small-size crease.
   w=(std::max)(0.,(std::min)(20.,w));
   c.emplace_back(a,a,w);c.emplace_back(b,b,w);c.emplace_back(a,b,-w);c.emplace_back(b,a,-w);area[ids[j]]+=twice/6;
  }
 }
 Sp C(ucCount,ucCount);C.setFromTriplets(c.begin(),c.end());
 std::vector<Triplet> inverse,diagonal;mask.resize(ucCount);screen.resize(ucCount);
 float growth=Smoother01((radius-2.9f)/4.72f);
 for(unsigned i=0;i<ucCount;i++){
  V3 p=Point(before,i),q=p-root;float s=Dot(q,axis),y=q.y,z=Dot(q,up),rho=sqrtf(y*y+z*z),upper=(z/max(rho,1e-8f)+1)*.5f;
  float reach=5+growth*(2.5f+2.5f*upper);
  double w=Smoother01((s+reach)/3)*(1-Smoother01((s/length-.12f)/.26f));
  w*=1-Smoother01((rho-(radius*1.55f+2))/3);
  float ventralReach=4+4*Smoother01((radius-2.7f)/1.1f);
  w*=Smoother01((p.x-(2+ventralReach*(1-upper)))/3);
  w*=Smoother01((p.z-(root.z-radius*2.1f-2))/4);
  mask[i]=w;area[i]=(std::max)(area[i],.005);screen[i]=area[i]*(2.5+2*pow(1-w,4));
  inverse.emplace_back(i,i,1/area[i]);diagonal.emplace_back(i,i,screen[i]);
 }
 Sp D(ucCount,ucCount),S(ucCount,ucCount);D.setFromTriplets(inverse.begin(),inverse.end());S.setFromTriplets(diagonal.begin(),diagonal.end());
 Sp CP=C*P;Sp K=CP.transpose()*D*CP*8.+P.transpose()*C*P*2.+P.transpose()*S*P;
 freeIDs.clear();fixedIDs.clear();freeOf.assign(masters.size(),-1);fixedOf.assign(masters.size(),-1);
 for(unsigned j=0;j<masters.size();j++)if(mask[masters[j]]>1e-4){freeOf[j]=(int)freeIDs.size();freeIDs.push_back(j);}else{fixedOf[j]=(int)fixedIDs.size();fixedIDs.push_back(j);}
 std::vector<bool> write(ucCount,false);for(unsigned j:freeIDs)write[masters[j]]=true;
 for(unsigned k=0;k<ucSeamCount;k++)write[ucSeamVertices[k*3]]=write[ucSeamVertices[k*3+1]]||write[ucSeamVertices[k*3+2]];
 normalFaces.clear();packedIDs.clear();
 for(unsigned k=0;k<ucFaceCount;k++)if(write[ucFaces[k*3]]||write[ucFaces[k*3+1]]||write[ucFaces[k*3+2]])normalFaces.push_back(k);
 for(unsigned i=0;i<ucPackedCount;i++)if(write[ucMap[i]])packedIDs.push_back(i);
 std::vector<Triplet> aa,ab;
 for(int col=0;col<K.outerSize();col++)for(Sp::InnerIterator it(K,col);it;++it){int row=freeOf[it.row()];if(row<0)continue;if(freeOf[it.col()]>=0)aa.emplace_back(row,freeOf[it.col()],it.value());else ab.emplace_back(row,fixedOf[it.col()],it.value());}
 A.resize((int)freeIDs.size(),(int)freeIDs.size());A.setFromTriplets(aa.begin(),aa.end());fixedCoupling.resize((int)freeIDs.size(),(int)fixedIDs.size());fixedCoupling.setFromTriplets(ab.begin(),ab.end());
 // Compile only the free rows of P' S and their nonzero fixed boundary.
 // The global metric remains unchanged; no geometry or constraints are dropped.
 targetIDs.clear();for(unsigned i=0;i<ucCount;i++)if(mask[i]>0)targetIDs.push_back(i);
 projectionOffsets.clear();projectionSources.clear();projectionWeights.clear();
 boundaryOffsets.clear();boundarySources.clear();boundaryWeights.clear();
 Eigen::SparseMatrix<double,Eigen::RowMajor> boundary=fixedCoupling;
 for(unsigned j=0;j<freeIDs.size();j++){
  projectionOffsets.push_back((unsigned)projectionSources.size());
  for(Sp::InnerIterator it(P,freeIDs[j]);it;++it){projectionSources.push_back((unsigned)it.row());projectionWeights.push_back(it.value());}
  boundaryOffsets.push_back((unsigned)boundarySources.size());
  for(Eigen::SparseMatrix<double,Eigen::RowMajor>::InnerIterator it(boundary,j);it;++it){boundarySources.push_back(masters[fixedIDs[it.col()]]);boundaryWeights.push_back(it.value());}
 }
 projectionOffsets.push_back((unsigned)projectionSources.size());boundaryOffsets.push_back((unsigned)boundarySources.size());
 factor.compute(A);valid=factor.info()==Eigen::Success;
 if(valid)inverseDiagonal=factor.vectorD().cwiseInverse();
 buildMS=PerfClock()-begin;builds++;
}
// Solve the same LDLT system for XYZ together. Eigen's sparse triangular
// path traverses the factor once per column and copies row-major inputs.
// Keeping each point's three coordinates adjacent reuses each factor entry.
static Mat SolveXYZ(const Mat& rhs){
 Mat value=factor.permutationP()*rhs;
 const Sp& lower=factor.matrixL().nestedExpression();
 const int* offsets=lower.outerIndexPtr();const int* rows=lower.innerIndexPtr();
 const double* weights=lower.valuePtr();double* xyz=value.data();
 const int count=(int)value.rows();
 for(int col=0;col<count;col++){
  const __m128d xy=_mm_loadu_pd(xyz+col*3);const double z=xyz[col*3+2];
  for(int k=offsets[col];k<offsets[col+1];k++){
   double* dest=xyz+rows[k]*3;const double w=weights[k];
   _mm_storeu_pd(dest,_mm_sub_pd(_mm_loadu_pd(dest),_mm_mul_pd(xy,_mm_set1_pd(w))));dest[2]-=z*w;
  }
 }
 for(int i=0;i<count;i++){const double inverse=inverseDiagonal[i];xyz[i*3]*=inverse;xyz[i*3+1]*=inverse;xyz[i*3+2]*=inverse;}
 for(int col=count-1;col>=0;col--){
  __m128d xy=_mm_loadu_pd(xyz+col*3);double z=xyz[col*3+2];
  for(int k=offsets[col];k<offsets[col+1];k++){
   const double* source=xyz+rows[k]*3;const double w=weights[k];xy=_mm_sub_pd(xy,_mm_mul_pd(_mm_set1_pd(w),_mm_loadu_pd(source)));z-=w*source[2];
  }
  _mm_storeu_pd(xyz+col*3,xy);xyz[col*3+2]=z;
 }
 return factor.permutationPinv()*value;
}
static void Apply(unsigned char* body){
 PerfScope perf(14);
 Topology();Read(body);V3 root,axis;SampleShaftChain(0,root,axis);V3 up=Unit(Cross(axis,{0,1,0}));float radius=logicalShaftBodyRadius,length=0;V3 previous=root;
 for(unsigned k=1;k<=100;k++){V3 point,tangent;SampleShaftChain(k*.01f,point,tangent);length+=Length(point-previous);previous=point;}length=max(.01f,length);
 bool controlChanged=cachedControls[7]!=hangUI||cachedControls[8]!=glansUI;
 for(unsigned j=0;j<7;j++)controlChanged|=cachedControls[j]!=sliderUI[j];
  int nextSequencePhase=teachingTimeline.active&&teachingTimeline.time>=6.3?1:0;
  bool phaseChanged=valid&&nextSequencePhase!=metricSequencePhase;
  Mat priorPhaseShape;if(phaseChanged&&solved.rows()==ucCount)priorPhaseShape=solved;
  bool rebuild=!valid||mode!=physicsState||controlChanged||phaseChanged;
  // Both ambient throbs and the J timeline animate effective shape values.
  // Keep solving fresh targets each frame, but reuse the metric through either
  // pulse. Large dimension changes and explicit controls still rebuild it.
  if(revision!=geometryRestRevision)rebuild|=(!throbMode&&!teachingTimeline.active)||fabsf(radius-cachedRadius)>cachedRadius*.35f||fabsf(length-cachedLength)>cachedLength*.35f;
  if(rebuild){Build(root,axis,up,radius,length);mode=physicsState;metricSequencePhase=nextSequencePhase;cachedRadius=radius;cachedLength=length;for(unsigned j=0;j<7;j++)cachedControls[j]=sliderUI[j];cachedControls[7]=hangUI;cachedControls[8]=glansUI;}
 revision=geometryRestRevision;
 if(!valid)return;double start=PerfClock();target=before;
 float growth=Smoother01((radius-2.9f)/4.72f);
 for(unsigned i:targetIDs){
  V3 p=Point(before,i),q=p-root;float s=Dot(q,axis),y=q.y,z=Dot(q,up),radial=sqrtf(y*y+z*z);if(radial<1e-8f)continue;
  float dy=y/radial,dz=z/radial;
  // Use the existing guide, enlarged only through the proximal transition.
  float parameter=s/length;auto frame=RapheChainFrame(parameter);float tubeR=RapheRadius(parameter)*(1+1-Smoother01((parameter-.04f)/.34f));
  V3 center=frame.p+frame.down*RapheDepth(parameter);float cy=(center-root).y,cz=Dot(center-root,up);
  float dot=dy*cy+dz*cz,disc=tubeR*tubeR-cy*cy-cz*cz+dot*dot;
  float bottom=disc>0&&dot>0?dot+sqrtf(max(0.f,disc)):0;
  float barrel=radius*1.025f+(.06f+.12f*growth)*radius*(1-Smoother01((parameter+.03f)/.26f));
  float gap=max(0.f,max(barrel,bottom)-radial)*(float)mask[i];V3 delta=(V3{0,1,0}*dy+up*dz)*gap;
  // The ventral expansion must not push the attachment back into the thighs.
  // Blend to the forward half-space through the lower arc of the section.
  delta.x=max(delta.x,delta.x*(1-Smoother01((-dz-.05f)/.60f)));
  target.row(i)+=Eigen::RowVector3d(delta.x,delta.y,delta.z);
 }
 profile[0]=PerfClock()-start;double mark=PerfClock();
 static Mat rhs;rhs.resize((int)freeIDs.size(),3);
 for(unsigned j=0;j<freeIDs.size();j++){
  Eigen::RowVector3d projected=Eigen::RowVector3d::Zero(),boundary=Eigen::RowVector3d::Zero();
  for(unsigned k=projectionOffsets[j];k<projectionOffsets[j+1];k++){unsigned i=projectionSources[k];projected+=(target.row(i)*screen[i])*projectionWeights[k];}
  for(unsigned k=boundaryOffsets[j];k<boundaryOffsets[j+1];k++)boundary+=before.row(boundarySources[k])*boundaryWeights[k];
  rhs.row(j)=projected-boundary;
 }
 profile[1]=PerfClock()-mark;mark=PerfClock();
 Mat result=SolveXYZ(rhs);if(factor.info()!=Eigen::Success||!result.allFinite())return;
#ifdef COLLAR_VERIFY_SOLVE
 // Test-only comparison against the preceding full-domain Eigen path.
 Mat weighted=target;for(unsigned i=0;i<ucCount;i++)weighted.row(i)*=screen[i];Mat projected=P.transpose()*weighted;
 Mat fixed((int)fixedIDs.size(),3),referenceRhs((int)freeIDs.size(),3);
 for(unsigned j=0;j<fixedIDs.size();j++)fixed.row(j)=before.row(masters[fixedIDs[j]]);
 for(unsigned j=0;j<freeIDs.size();j++)referenceRhs.row(j)=projected.row(freeIDs[j]);referenceRhs-=fixedCoupling*fixed;
 Mat reference=factor.solve(referenceRhs);double error=(reference-result).cwiseAbs().maxCoeff();
 if(error>1e-9){fprintf(stderr,"collar solve equivalence failed: %.12g\n",error);abort();}
#endif
 profile[2]=PerfClock()-mark;mark=PerfClock();
 solved=before;
 for(unsigned j=0;j<freeIDs.size();j++)solved.row(masters[freeIDs[j]])=result.row(j);
 for(unsigned k=0;k<ucSeamCount;k++){unsigned x=ucSeamVertices[k*3],a=ucSeamVertices[k*3+1],b=ucSeamVertices[k*3+2];double u=ucSeamWeights[k];solved.row(x)=solved.row(a)*(1-u)+solved.row(b)*u;}
 if(phaseChanged&&priorPhaseShape.rows()==ucCount){transitionCorrection=priorPhaseShape-solved;transitionFrames=8;}
 if(transitionFrames){solved+=transitionCorrection*(double(transitionFrames)/8.);transitionFrames--;}
 static std::vector<V3> normals(ucCount);std::fill(normals.begin(),normals.end(),V3{});
 for(unsigned k:normalFaces){unsigned a=ucFaces[k*3],b=ucFaces[k*3+1],c=ucFaces[k*3+2];V3 n=Cross(Point(solved,b)-Point(solved,a),Point(solved,c)-Point(solved,a));normals[a]=normals[a]+n;normals[b]=normals[b]+n;normals[c]=normals[c]+n;}
 profile[3]=PerfClock()-mark;mark=PerfClock();
 for(unsigned i:packedIDs){
  unsigned j=ucMap[i];V3 p=Point(solved,j);unsigned char* dst=Packed(i,body);memcpy(dst,&p,12);
  if(mask[j]>1e-4){V3 n=Unit(normals[j]),t{dst[12]/255.f*2-1,dst[13]/255.f*2-1,dst[14]/255.f*2-1};t=Unit(t-n*Dot(n,t));dst[12]=PackSigned(t.x);dst[13]=PackSigned(t.y);dst[14]=PackSigned(t.z);dst[16]=PackSigned(n.x);dst[17]=PackSigned(n.y);dst[18]=PackSigned(n.z);}
  if(i<nrCount)nrPositions[i]=p;
  else if(i-nrCount<sharedBodyCount[0])memcpy(sharedBodyOutput[0]+(i-nrCount)*32,dst,32);
  else memcpy(sharedBodyOutput[1]+(i-nrCount-sharedBodyCount[0])*32,dst,32);
 }
 profile[4]=PerfClock()-mark;solveMS=PerfClock()-start;
}
}
