#include <malemod/physics/rigid_cluster.hpp>
#include <cstdio>
using namespace malemod;using namespace malemod::physics;
int main(){
 State s;s.position={{-.04f,0,0},{-.01f,.001f,0},{.015f,.002f,0},{.04f,.003f,0},
  {0,0,0},{.03f,0,0},{0,.04f,0},{0,0,.05f}};
 auto rest=s.position;V3 rc=ClusterCenter(s,0,4,4,4);
 float maxFit=0,maxShape=0;
 for(int axis=0;axis<3;axis++){
  Q4 previous={0,0,0,1};
  for(int step=0;step<180;step++){
   float a=step*.02f;Q4 expected={0,0,0,cosf(a*.5f)};
   if(axis==0)expected.x=sinf(a*.5f);if(axis==1)expected.y=sinf(a*.5f);if(axis==2)expected.z=sinf(a*.5f);
   V3 origin={.1f,-.03f,.07f};
   for(int i=0;i<8;i++)s.position[i]=origin+RotateCluster(expected,rest[i]-rc);
   Q4 q=FitCluster(s,rest,rc,0,4,4,4,previous);previous=q;
   for(int i=0;i<8;i++)maxFit=max(maxFit,Length(RotateCluster(q,rest[i]-rc)-RotateCluster(expected,rest[i]-rc)));
   s.position[0]=s.position[0]+V3{.003f,-.002f,.001f};
   q=FitCluster(s,rest,rc,0,4,4,4,q);ProjectCluster(s,rest,rc,0,4,4,4,q);
   for(int i=0;i<8;i++)for(int j=0;j<i;j++)maxShape=max(maxShape,fabsf(Length(s.position[i]-s.position[j])-Length(rest[i]-rest[j])));
  }
 }
 if(maxFit>1e-5f||maxShape>1e-6f)return 2;
 printf("{\"threeAxes\":true,\"frames\":540,\"maximumFitError\":%.9g,\"maximumPairDistanceError\":%.9g}\n",maxFit,maxShape);
}
