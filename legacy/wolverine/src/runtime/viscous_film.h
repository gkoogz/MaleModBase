#pragma once
// Conservative shallow surface film in a receiver tangent plane. This is an
// illustrative wetting model, not a calibrated 3D free-surface solver.
namespace volumeFluid {
struct FilmCell {float mass=0,mx=0,my=0;};
struct ViscousFilm {
 static constexpr int nx=160,ny=96;static constexpr float dx=.4f;
 std::vector<FilmCell> cells,next;int x0=nx,y0=ny,x1=-1,y1=-1;double deposited=0,clock=0;
 ViscousFilm():cells(nx*ny),next(nx*ny){}
 int Index(int x,int y)const{return y*nx+x;}
 float X(int x)const{return (x-24)*dx;}float Y(int y)const{return (y-ny/2)*dx;}
 float Height(int x,int y)const{return x>=0&&x<nx&&y>=0&&y<ny?cells[Index(x,y)].mass/(dx*dx):0;}
 bool Fits(float x,float y)const{return x>X(4)&&x<X(nx-5)&&y>Y(4)&&y<Y(ny-5);}
 void Deposit(float x,float y,float volume,float vx,float vy,float normalSpeed){
  if(volume<=0||!Fits(x,y))return;
  float radius=max(.55f,min(2.2f,.45f+.25f*cbrtf(volume)+normalSpeed*.003f));
  int cx=int(x/dx+24),cy=int(y/dx+ny/2),r=int(ceilf(radius*2/dx));
  double weights=0;for(int j=max(1,cy-r);j<=min(ny-2,cy+r);j++)for(int i=max(1,cx-r);i<=min(nx-2,cx+r);i++){float a=(X(i)-x)/radius,b=(Y(j)-y)/radius;weights+=expf(-2*(a*a+b*b));}
  for(int j=max(1,cy-r);j<=min(ny-2,cy+r);j++)for(int i=max(1,cx-r);i<=min(nx-2,cx+r);i++){
   float a=(X(i)-x)/radius,b=(Y(j)-y)/radius,w=float(expf(-2*(a*a+b*b))/weights),mass=volume*w;
   auto& c=cells[Index(i,j)];c.mass+=mass;
   // Only tangential momentum survives sticking to the receiver. Normal
   // impact energy gives a bounded outward spread, not arbitrary random dots.
   float radial=min(4.f,normalSpeed*.025f);
   c.mx+=mass*(vx*.32f+a*radial);c.my+=mass*(vy*.32f+b*radial);
  }
  x0=min(x0,max(1,cx-r));x1=max(x1,min(nx-2,cx+r));y0=min(y0,max(1,cy-r));y1=max(y1,min(ny-2,cy+r));deposited+=volume;
 }
 void Step(float dt,float viscosity,float tension){
  if(x1<x0)return;int loX=max(1,x0-2),hiX=min(nx-2,x1+2),loY=max(1,y0-2),hiY=min(ny-2,y1+2);
  auto pressure=[&](int x,int y){float h=Height(x,y),lap=(Height(x-1,y)+Height(x+1,y)+Height(x,y-1)+Height(x,y+1)-4*h)/(dx*dx);return 4.f*h-tension*.008f*lap;};
  float drag=expf(-dt*(1.6f+viscosity*.07f));
  for(int y=loY;y<=hiY;y++)for(int x=loX;x<=hiX;x++){
   auto& c=cells[Index(x,y)];if(c.mass<=1e-12f)continue;
   float vx=c.mx/c.mass,vy=c.my/c.mass;
   vx-=(pressure(x+1,y)-pressure(x-1,y))/(2*dx)*dt;vy-=(pressure(x,y+1)-pressure(x,y-1))/(2*dx)*dt;
   float speed=sqrtf(vx*vx+vy*vy),limit=dx*.7f/dt;if(speed>limit){vx*=limit/speed;vy*=limit/speed;}
   c.mx=c.mass*vx*drag;c.my=c.mass*vy*drag;
  }
  // Symmetric pairwise velocity diffusion conserves film momentum before
  // substrate friction. The harmonic mass prevents thin edges dragging pools.
  float mix=min(.2f,dt*viscosity*.12f/(dx*dx));
  for(int y=loY;y<=hiY;y++)for(int x=loX;x<=hiX;x++)for(int axis=0;axis<2;axis++){
   int xx=x+(axis==0),yy=y+(axis==1);if(xx>hiX||yy>hiY)continue;auto& a=cells[Index(x,y)];auto& b=cells[Index(xx,yy)];
   if(a.mass<1e-10f||b.mass<1e-10f)continue;float m=mix*a.mass*b.mass/(a.mass+b.mass);
   float px=m*(b.mx/b.mass-a.mx/a.mass),py=m*(b.my/b.mass-a.my/a.mass);a.mx+=px;a.my+=py;b.mx-=px;b.my-=py;
  }
  for(int y=loY-1;y<=hiY+1;y++)for(int x=loX-1;x<=hiX+1;x++)next[Index(x,y)]={};
  for(int y=loY;y<=hiY;y++)for(int x=loX;x<=hiX;x++){
   auto c=cells[Index(x,y)];if(c.mass<=0)continue;float fx=c.mx/c.mass*dt/dx,fy=c.my/c.mass*dt/dx;
   if(x<=1&&fx<0||x>=nx-2&&fx>0)fx=0;if(y<=1&&fy<0||y>=ny-2&&fy>0)fy=0;
   float total=fabsf(fx)+fabsf(fy);if(total>.8f){fx*=.8f/total;fy*=.8f/total;total=.8f;}
   auto add=[&](int xx,int yy,float w){auto& n=next[Index(xx,yy)];n.mass+=c.mass*w;n.mx+=c.mx*w;n.my+=c.my*w;};
   add(x,y,1-total);if(fx)add(x+(fx>0?1:-1),y,fabsf(fx));if(fy)add(x,y+(fy>0?1:-1),fabsf(fy));
  }
  x0=nx;y0=ny;x1=y1=-1;
  for(int y=loY-1;y<=hiY+1;y++)for(int x=loX-1;x<=hiX+1;x++){
   auto& c=cells[Index(x,y)];c=next[Index(x,y)];if(c.mass>1e-12f){x0=min(x0,x);x1=max(x1,x);y0=min(y0,y);y1=max(y1,y);}
  }
  clock+=dt;
 }
 double Mass()const{double sum=0;for(const auto& c:cells)sum+=c.mass;return sum;}
};
}
