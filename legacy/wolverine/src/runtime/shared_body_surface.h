#pragma once
#include "menu_retarget_body_data.h"
// Canonical authored body, published after the same evaluator in either scene.
// Positions and tangent basis are shared; compact skin palettes remain adapters.
static unsigned char menuRetargetBodyDynamic0[sizeof(menuRetargetBodyPacked0)];
static unsigned char menuRetargetBodyDynamic1[sizeof(menuRetargetBodyPacked1)];
static bool menuRetargetBodyDynamicReady=false,menuRetargetBodyUploadPending[2]={true,true};
static unsigned sharedBodyRevision=0;

static const unsigned sharedBodyFirst[2]={17449,41435};
static const unsigned sharedBodyCount[2]={menuRetargetBodyVertexCount0,menuRetargetBodyVertexCount1};
static const unsigned char* sharedBodyBase[2]={menuRetargetBodyPacked0,menuRetargetBodyPacked1};
static unsigned char* sharedBodyOutput[2]={menuRetargetBodyDynamic0,menuRetargetBodyDynamic1};
static bool MenuRetargetBodyVertex(unsigned source,int& section,UINT& local){
 for(int s=0;s<2;s++)if(source>=sharedBodyFirst[s]&&source<sharedBodyFirst[s]+sharedBodyCount[s]){section=s;local=source-sharedBodyFirst[s];return true;}return false;
}
static void EnsureSharedBody(){
 if(menuRetargetBodyDynamicReady)return;
 for(int s=0;s<2;s++)memcpy(sharedBodyOutput[s],sharedBodyBase[s],sharedBodyCount[s]*32);
 menuRetargetBodyDynamicReady=true;
}
static void PrepareMenuRetargetBodyBasis(unsigned char* donor){
 EnsureSharedBody();
 for(int s=0;s<2;s++)memcpy(donor+sharedBodyFirst[s]*32,sharedBodyBase[s],sharedBodyCount[s]*32);
}
static void ResetSharedBodySurface(unsigned char* buffer){
 EnsureSharedBody();
 for(int s=0;s<2;s++)for(unsigned i=0;i<sharedBodyCount[s];i++)memcpy(buffer+(sharedBodyFirst[s]+i)*32,sharedBodyBase[s]+i*32,20);
}
// Future body sculpt/physics fields write final positions and tangent bases here.
// This one hook feeds BOTH renderers, collision, and accessory anchors; it must
// operate on the reset model-space surface, never on the preceding frame.
static void (*sharedBodyDeform)(unsigned char* buffer,unsigned char* anatomy)=nullptr;
static void UpdateMenuRetargetBodyWeld(const unsigned char* donor){
 EnsureSharedBody();
 for(int s=0;s<2;s++){
  bool changed=false;
  for(unsigned i=0;i<sharedBodyCount[s];i++){
   auto* dst=sharedBodyOutput[s]+i*32;const auto* src=donor+(sharedBodyFirst[s]+i)*32;
   if(memcmp(dst,src,20)){memcpy(dst,src,20);changed=true;}
  }
  menuRetargetBodyUploadPending[s]|=changed;
  if(changed)++sharedBodyRevision;
 }
}
