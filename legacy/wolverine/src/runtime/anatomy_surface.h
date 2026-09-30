#pragma once
#include "unified_collar_solver.h"
// One evaluator for gameplay and WStart. Scene adapters own buffers and bone
// palettes; morphology, simulation output, collar and normal construction do not.
static void EvaluateAnatomy(unsigned char* fullBuffer,UINT graftFirstVertex){
  ResetSharedBodySurface(fullBuffer);
  auto* controlled=fullBuffer+pelvisControlFirstVertex*graftStride;
  auto* p=controlled+(graftFirstVertex-pelvisControlFirstVertex)*graftStride;
  V3 tipSum{};int tipCount=0;
  PrepareShape(controlled,graftFirstVertex,tipSum,tipCount);
  if(!constraintSolverReady)InitializeConstraintSolver();
  for(UINT i=0;i<graftCount;i++){
    float value[3]={graftDeformedPositions[i].x,graftDeformedPositions[i].y,graftDeformedPositions[i].z};
    float bw=phys_scrotum_weight[i],membership=max(phys_shaft_weight[i],phys_attachment_weight[i]);float motionWeight=membership;if(bw<.05f&&membership>.02f)motionWeight=1.f;
    if(suspensionWeight[i]>0.f)ApplySuspendedSkin(value,i);
    else if(motionWeight>.001f)ApplyConstraintCurve(value,i);
    graftDeformedPositions[i]=V3{value[0],value[1],value[2]};
  }
  // Reassert the one shaft after every physics state.  This transports the
  // cached complete cross-section through the live centerline and prevents a
  // mixed-weight ring from reappearing in semi/floppy motion.
  ConstructLogicalShaftSurface(true);
  // Keep the distal shaft independent of skin fairing. The proximal .40
  // belongs to the shared pelvic ramp, whose support fades at .40. Restoring
  // the old tube at .04-.10 would undo that ramp and recreate the shelf.
  static V3 solidCore[graftCount];memcpy(solidCore,graftDeformedPositions,sizeof(solidCore));
  FinishScrotalJunction();
  for(UINT i=0;i<graftCount;i++){
    if(suspensionWeight[i]!=0.f||max(phys_shaft_weight[i],phys_attachment_weight[i])<.5f)continue;
    float follow=Smoother01((graftRestFlex[i]-.30f)/.10f);
    graftDeformedPositions[i]=graftDeformedPositions[i]*(1.f-follow)+solidCore[i]*follow;
  }
  SculptVentralContourStudy();
  // Glans Size is evaluated on the approved R14 mesh, not on the donor cage.
  ResolveSuspendedSkinContact();
  PreserveEggSupports();
  // Proximal skin is now owned by UnifiedCollar, including pelvic recruitment.
  // The proxy changes vertex positions after UE3 has prepared the skeletal
  // buffer.  Rebuild the normals from that final deformed surface so lighting
  // follows every physics bend.  Wolverine's meshes use the opposite of the
  // raw index-buffer cross-product; the topology header also welds UV-split
  // duplicates into shared smoothing groups.
  memset(graftDynamicNormalSums,0,sizeof(graftDynamicNormalSums));
  memset(graftDynamicTangentSums,0,sizeof(graftDynamicTangentSums));
  memset(collarTangentSums,0,sizeof(collarTangentSums));
  for(UINT k=0;k<graftTriangleIndexCount;k+=3){
    UINT ia=graftTriangleIndices[k],ib=graftTriangleIndices[k+1],ic=graftTriangleIndices[k+2];
    V3 edge1=graftDeformedPositions[ib]-graftDeformedPositions[ia],edge2=graftDeformedPositions[ic]-graftDeformedPositions[ia];
    V3 face=Cross(edge2,edge1);
    UINT ga=graftNormalGroup[ia],gb=graftNormalGroup[ib],gc=graftNormalGroup[ic];
    graftDynamicNormalSums[ga]=graftDynamicNormalSums[ga]+face;
    graftDynamicNormalSums[gb]=graftDynamicNormalSums[gb]+face;
    graftDynamicNormalSums[gc]=graftDynamicNormalSums[gc]+face;
    float du1=graftUVs[ib*2]-graftUVs[ia*2],dv1=graftUVs[ib*2+1]-graftUVs[ia*2+1];
    float du2=graftUVs[ic*2]-graftUVs[ia*2],dv2=graftUVs[ic*2+1]-graftUVs[ia*2+1],det=du1*dv2-dv1*du2;
    if(fabsf(det)>1e-8f){
      V3 tangent=(edge1*dv2-edge2*dv1)/det;graftDynamicTangentSums[ia]=graftDynamicTangentSums[ia]+tangent;graftDynamicTangentSums[ib]=graftDynamicTangentSums[ib]+tangent;graftDynamicTangentSums[ic]=graftDynamicTangentSums[ic]+tangent;
      V3 direction=Unit(tangent);UINT groups[3]={graftCollarGroups[ia],graftCollarGroups[ib],graftCollarGroups[ic]};
      for(int q=0;q<3;q++)if(groups[q]!=65535u)collarTangentSums[groups[q]]=collarTangentSums[groups[q]]+direction;
    }
  }
  RebuildUnifiedCollarNormals(controlled,graftFirstVertex);
  RebuildUnifiedCollarTangents();
  for(UINT i=0;i<graftCount;i++){
    memcpy(p+i*graftStride,&graftDeformedPositions[i],12);
    UINT collarGroup=graftCollarGroups[i];
    V3 dynamic=collarGroup!=65535u?collarNormalSums[collarGroup]:Unit(graftDynamicNormalSums[graftNormalGroup[i]]);
    V3 baseNormal={graftBaseNormals[i*3],graftBaseNormals[i*3+1],graftBaseNormals[i*3+2]};
    float lock=collarGroup!=65535u?0.f:graftSeamNormalLock[i];V3 normal=Unit(dynamic*(1.f-lock)+baseNormal*lock);
    V3 tangent=collarGroup!=65535u?collarTangentSums[collarGroup]:graftDynamicTangentSums[i]-normal*Dot(normal,graftDynamicTangentSums[i]);tangent=Unit(tangent);
    unsigned char* packedTangent=p+i*graftStride+12;unsigned char* packedNormal=p+i*graftStride+16;
    packedTangent[0]=PackSigned(tangent.x);packedTangent[1]=PackSigned(tangent.y);packedTangent[2]=PackSigned(tangent.z);
    packedNormal[0]=PackSigned(normal.x);packedNormal[1]=PackSigned(normal.y);packedNormal[2]=PackSigned(normal.z);
  }
  UpdateR14(p);
  ApplyPelvicAttachment(fullBuffer);
  ApplyRoundedShape(fullBuffer);
  if(sharedBodyDeform){
    sharedBodyDeform(fullBuffer,rsPacked);
    // Keep the authored boundary welded even when a future body field touches
    // the pelvis. The field receives both surfaces for a continuous transition.
    for(unsigned k=0;k<paSeamCount;k++)memcpy(rsPacked+paSeamR14[k]*32,fullBuffer+paBodyNormalIDs[paSeamBody[k]]*32,12);
  }
  UpdateMenuRetargetBodyWeld(fullBuffer);
  UpdateNeckRender();
  UnifiedCollar::Apply(fullBuffer);
  if(tipCount){V3 tip=tipSum/(float)tipCount;float newLength=max(8.f,min(60.f,Length(tip-ShaftRoot())));constraintRestLength=(constraintRestLength*.1656f+newLength*.08f)/.2456f;}
}
