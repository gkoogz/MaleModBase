#pragma once
namespace detail {
struct BandRest {std::vector<std::vector<Sample>> measured;double height=0,gap=0;};
inline BandRest BuildBand(const Input& input,const Parameters& parameters,Output& output){
 const double circumference=output.measuredCircumference,width=parameters.bandWidth*circumference,thick=parameters.bandThickness*circumference,gap=parameters.clearance*circumference;
 // Native route samples identify the body; they are not a limit on elastic
 // silhouette quality. Densify the exact cut itself, retaining native donors,
 // and let consumers use the exported band layout for material indices.
 auto& m=output.mesh;const auto& frame=input.frame;const unsigned waist=input.bodySurface.empty()?unsigned(input.waist.size()):(std::max)(96u,unsigned(input.waist.size()));
    // A closed thick waistband with separate thin stripe bands, not a flat
    // image decal. Duplicate seams retain equal positions and original donors.
    double waistHeight=0;for(auto sample:input.waist)waistHeight+=frame.Local(sample.position)[2];waistHeight/=input.waist.size();
    auto seating=band_seating::Guide(input,width);
    auto bandAnchor=[&](Point p,double row){auto local=frame.Local(p);local[2]=waistHeight+seating.Offset(local,row);return frame.World(local);};
    const std::array<double,7> rows{{-.5,-.10,-.045,.045,.10,.39,.5}};
    std::vector<std::vector<Sample>> measuredBand;
    const double bandGap=input.bodySurface.empty()?gap:gap*.3;
    if(!input.bodySurface.empty()){
      waist::JoinedSurface joined;const auto* cutSamples=&input.bodySurface;const auto* cutFaces=&input.bodyTriangles;
      if(!input.anatomyRegions[0].empty()){joined=waist::JoinedPhysicalSurface(input,circumference);cutSamples=&joined.samples;cutFaces=&joined.triangles;}
      auto middle=waist::detail::Slice(*cutSamples,*cutFaces,frame,waistHeight,waist,[&](Point p){return seating.Height(p);},&seating.center);
      band_seating::MeasureWidth(seating,middle,frame);
      waist::BandContours bounds{waist::detail::Slice(*cutSamples,*cutFaces,frame,waistHeight,waist,[&](Point p){return seating.Offset(p,-.5);},&seating.center),waist::detail::Slice(*cutSamples,*cutFaces,frame,waistHeight,waist,[&](Point p){return seating.Offset(p,.5);},&seating.center)};
      output.band.topAttachments=unsigned(bounds.top.points.size());output.band.bottomAttachments=unsigned(bounds.bottom.points.size());output.band.topCircumference=bounds.top.circumference;output.band.bottomCircumference=bounds.bottom.circumference;output.band.preferredClearance=bandGap;
      auto samples=[&](const auto& contour){std::vector<Sample> out;for(const auto& a:contour.points){Sample sample;sample.position=a.position;sample.normal=a.normal;std::vector<Donor> donors;for(unsigned k=0;k<4;k++)if(a.weights[k]>0)for(auto donor:(*cutSamples)[a.vertices[k]].lineage.donors)if(donor.weight>0){donor.weight*=a.weights[k];auto it=std::find_if(donors.begin(),donors.end(),[&](auto d){return d.surface==donor.surface&&d.vertex==donor.vertex;});if(it==donors.end())donors.push_back(donor);else it->weight+=donor.weight;}if(donors.size()>sample.lineage.donors.size())throw std::invalid_argument("Body contour attachment exceeds exact lineage contract");for(unsigned k=0;k<donors.size();k++)sample.lineage.donors[k]=donors[k];out.push_back(sample);}return out;};
      for(unsigned row=0;row<rows.size();row++){if(row==0)measuredBand.push_back(samples(bounds.bottom));else if(row==6)measuredBand.push_back(samples(bounds.top));else measuredBand.push_back(samples(waist::detail::Slice(*cutSamples,*cutFaces,frame,waistHeight,waist,[&](Point p){return seating.Offset(p,rows[row]);},&seating.center)));}
      // Each angular column is one finite-thickness material strip. Its
      // director follows the measured full-width radial/vertical profile,
      // including the recruited ramp. Independent local shading or triangle
      // normals can twist adjacent thickness rows through one another at the
      // graft weld. Exact skin cuts and original native weights stay intact.
      auto original=measuredBand;
      for(unsigned col=0;col<waist;col++){
       auto local=Sub(frame.Local(original[3][col].position),seating.center);auto radial=Unit(Add(Mul(frame.lateral,local[0]),Mul(frame.forward,local[1])));
       auto tangent=Cross(frame.up,radial),across=Sub(original.back()[col].position,original.front()[col].position);
       auto normal=Cross(tangent,across);if(Length(normal)<circumference*circumference*1e-14)throw std::invalid_argument("Measured band has a degenerate material column");normal=Unit(normal);if(Dot(normal,radial)<0)normal=Mul(normal,-1);
       for(unsigned row=0;row<rows.size();row++)measuredBand[row][col].normal=normal;
      }
    }
    for(unsigned layer=0;layer<2;layer++){
        const unsigned base=unsigned(m.vertices.size());
        for(unsigned j=0;j<rows.size();j++)for(unsigned i=0;i<=waist;i++){const auto& s=measuredBand.empty()?input.waist[i%waist]:measuredBand[j][i%waist];auto normal=measuredBand.empty()?Sub(s.normal,Mul(frame.up,Dot(s.normal,frame.up))):s.normal;if(Length(normal)<1e-10){auto p=frame.Local(s.position);normal=Add(Mul(frame.lateral,p[0]),Mul(frame.forward,p[1]));}normal=Unit(normal);VertexAt(m,Add(measuredBand.empty()?bandAnchor(s.position,rows[j]):s.position,Mul(normal,bandGap+(layer?0:thick))),double(i)/waist,double(j)/6,s.lineage);}
        for(unsigned j=0;j+1<rows.size();j++)for(unsigned i=0;i<waist;i++){unsigned a=base+j*(waist+1)+i;auto normal=measuredBand.empty()?input.waist[i].normal:measuredBand[j][i].normal;if(layer)normal=Mul(normal,-1);MaterialSlot material=MaterialSlot::WhiteElastic;if(!layer&&j==1)material=MaterialSlot::RedStripe;if(!layer&&j==3)material=MaterialSlot::BlueStripe;Quad(m,a,a+1,a+waist+2,a+waist+1,material,normal);}
    }
    // Close upper/lower physical band edges. Separate normals preserve the
    // narrow thickness instead of a zero-volume strip.
    const unsigned back=7*(waist+1);for(unsigned row:{0u,6u})for(unsigned i=0;i<waist;i++){auto a=row*(waist+1)+i,b=a+1;Quad(m,a,b,b+back,a+back,MaterialSlot::WhiteElastic,Mul(frame.up,row?1:-1));}

 return {std::move(measuredBand),waistHeight,bandGap};
}
}
