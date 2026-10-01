static V3 CPSupportLocal(V3 n,V3 r){
 float h=sqrtf(n.x*n.x*r.x*r.x+n.y*n.y*r.y*r.y),v=n.z*r.z;
 if(h<1e-8f)return {0,0,copysignf(r.z,v)};
 // The tapered ellipse has strictly negative support-objective curvature.
 // Newton from the untapered ellipse converges without the quantization of
 // a 22-step float bisection, and is cheaper during normal refinement.
 double z=double(v)/sqrt(double(h)*h+double(v)*v),ratio=double(v)/h;
 z=max(-.999999999999,min(.999999999999,z));
 for(int i=0;i<6;i++){
  double radial=sqrt(max(1e-24,1.-z*z));
  double derivative=-.13*radial-(1.-.13*z)*z/radial+ratio;
  double curvature=(-1.+.39*z-.26*z*z*z)/(radial*radial*radial);
  z=max(-.999999999999,min(.999999999999,z-derivative/curvature));
 }
 float section=float((1.-.13*z)*sqrt(max(0.,1.-z*z)));
 return {r.x*r.x*n.x/h*section,r.y*r.y*n.y/h*section,float(r.z*z)};
}
