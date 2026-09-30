#include "scrotal_density_data.h"
static unsigned char densePacked[denseCount*32];
static void BuildDenseScrotalSurface(){
  memcpy(densePacked,r14Packed,sizeof(r14Packed));
  for(UINT edge=0;edge<denseEdgeCount;edge++){
    UINT a=denseEdges[edge*2],b=denseEdges[edge*2+1];
    unsigned char* v=densePacked+(r14Count+edge)*32;
    V3 p=(r14Positions[a]+r14Positions[b])*.5f;
    memcpy(v,&p,12);
    auto normal=[](UINT i){const unsigned char* q=r14Packed+i*32+16;return Unit(V3{q[0]/127.5f-1.f,q[1]/127.5f-1.f,q[2]/127.5f-1.f});};
    V3 n=Unit(normal(a)+normal(b));
    V3 t=Unit(r14Tangents[a]+r14Tangents[b]);t=Unit(t-n*Dot(t,n));
    if(Length(t)<1e-6f)t=Unit(Cross(fabsf(n.z)<.9f?V3{0,0,1}:V3{0,1,0},n));
    v[12]=PackSigned(t.x);v[13]=PackSigned(t.y);v[14]=PackSigned(t.z);v[15]=127;
    v[16]=PackSigned(n.x);v[17]=PackSigned(n.y);v[18]=PackSigned(n.z);v[19]=0;
    // Merge endpoint skin influences into the four slots the game shader uses.
    float influence[256]{};
    for(int k=0;k<4;k++){
      influence[r14Bones[a*4+k]]+=r14Weights[a*4+k]*.5f;
      influence[r14Bones[b*4+k]]+=r14Weights[b*4+k]*.5f;
    }
    float selected[4]{},total=0.f;
    for(int k=0;k<4;k++){
      int best=0;for(int bone=1;bone<256;bone++)if(influence[bone]>influence[best])best=bone;
      v[20+k]=(unsigned char)best;selected[k]=influence[best];total+=selected[k];influence[best]=0.f;
    }
    int remaining=255;
    for(int k=0;k<4;k++){
      int w=k==3?remaining:min(remaining,int(total>0.f?selected[k]*255.f/total+.5f:0.f));
      v[24+k]=(unsigned char)w;remaining-=w;
    }
    float uv[2]={(r14UV[a*2]+r14UV[b*2])*.5f,(r14UV[a*2+1]+r14UV[b*2+1])*.5f};
    D3DXFloat32To16Array(reinterpret_cast<D3DXFLOAT16*>(v+28),uv,2);
  }
}
