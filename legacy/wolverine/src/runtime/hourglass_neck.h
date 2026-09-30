// Broaden the shared upper pouch in material coordinates. C2 endpoint fades
// preserve the body weld and the existing lower support volumes.
static float hourglassAppliedFraction=1.f;
static void BroadenScrotalNeck(){
  static V3 delta[graftCount];
  V3 center{},axis{};SampleRestShaftFrame(.12f,center,axis);
  V3 lateral=Unit(V3{0,1,0}-axis*axis.y);
  for(int pass=0;pass<2;pass++){
  for(UINT i=0;i<graftCount;i++){
    float x=morph_base[i*3],z=morph_base[i*3+2];
    float blend=Smoother01((z-72.f)/3.f)*(1.f-Smoother01((z-79.f)/3.f))
      *Smoother01((x-9.f)/4.f)*(1.f-Smoother01((x-18.f)/5.f))
      *Smoother01(graftCollarDistances[i]/3.f);
    float front=Smoother01((morph_base[i*3]-11.5f)/4.5f);
    V3 p=graftDeformedPositions[i];
    delta[i]=pass==0?lateral*(Dot(p-center,lateral)*.65f*blend)
      :axis*(logicalShaftBodyRadius*.45f*front*blend);
  }
  float fraction=1.f;
  for(UINT k=0;k<graftTriangleIndexCount;k+=3){
    UINT a=graftTriangleIndices[k],b=graftTriangleIndices[k+1],c=graftTriangleIndices[k+2];
    fraction=min(fraction,SurfaceCorrectionLimit(graftDeformedPositions[a],graftDeformedPositions[b],graftDeformedPositions[c],delta[a],delta[b],delta[c],.40f));
  }
  hourglassAppliedFraction=fraction;
  for(UINT i=0;i<graftCount;i++)graftDeformedPositions[i]=graftDeformedPositions[i]+delta[i]*fraction;
  }
}
