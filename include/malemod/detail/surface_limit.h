// Continuous conservative bound on projected triangle area during a surface
// correction. For 0 <= s <= 1, A(s)/A(0) = 1 + b*s + a*s*s.
// Bounding only negative coefficients by -(negativeA+negativeB)*s avoids
// discontinuous power-of-two backtracking, while retaining the area floor.
static float SurfaceCorrectionLimit(V3 p0,V3 p1,V3 p2,V3 d0,V3 d1,V3 d2,float floor,float minimumArea2=1e-12f){
  V3 e1=p1-p0,e2=p2-p0,u=d1-d0,v=d2-d0,n=Cross(e1,e2);
  double area=double(n.x)*n.x+double(n.y)*n.y+double(n.z)*n.z;
  if(area<=minimumArea2)return 1.f;
  V3 quadratic=Cross(u,v),linear=Cross(u,e2)+Cross(e1,v);
  double a=(double(n.x)*quadratic.x+double(n.y)*quadratic.y+double(n.z)*quadratic.z)/area;
  double b=(double(n.x)*linear.x+double(n.y)*linear.y+double(n.z)*linear.z)/area;
  if(!std::isfinite(a)||!std::isfinite(b))return 0.f;
  double adverse=max(0.,-a)+max(0.,-b);
  return adverse>1.-floor?float((1.-floor)/adverse):1.f;
}

struct PreparedSurfaceLimit {
  V3 e1,e2,n;double area;
  void Prepare(V3 p0,V3 p1,V3 p2){
    e1=p1-p0;e2=p2-p0;n=Cross(e1,e2);
    area=double(n.x)*n.x+double(n.y)*n.y+double(n.z)*n.z;
  }
  float Evaluate(V3 d0,V3 d1,V3 d2,float floor,float minimumArea2=1e-12f) const {
    if(area<=minimumArea2)return 1.f;
    V3 u=d1-d0,v=d2-d0,quadratic=Cross(u,v),linear=Cross(u,e2)+Cross(e1,v);
    double a=(double(n.x)*quadratic.x+double(n.y)*quadratic.y+double(n.z)*quadratic.z)/area;
    double b=(double(n.x)*linear.x+double(n.y)*linear.y+double(n.z)*linear.z)/area;
    if(!std::isfinite(a)||!std::isfinite(b))return 0.f;
    double adverse=max(0.,-a)+max(0.,-b);
    return adverse>1.-floor?float((1.-floor)/adverse):1.f;
  }
};
