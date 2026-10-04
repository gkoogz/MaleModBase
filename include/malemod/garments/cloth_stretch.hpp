#pragma once
namespace malemod::garments::cloth_stretch {
template<class Edges> inline std::vector<std::vector<unsigned>> Adjacency(unsigned count,const Edges& edges){std::vector<std::vector<unsigned>> result(count);for(unsigned i=0;i<edges.size();i++)if(!edges[i].bend&&!edges[i].tether){result[edges[i].a].push_back(i);result[edges[i].b].push_back(i);}return result;}
struct Result{unsigned visits=0;bool exhausted=false;};
template<class Edges> inline Result Project(std::vector<Point>&x,const std::vector<double>&inverseMass,const Edges&edges,const std::vector<std::vector<unsigned>>&adjacency,double limit){
 std::queue<unsigned> queue;std::vector<unsigned char> scheduled(edges.size());auto add=[&](unsigned id){if(!scheduled[id]){scheduled[id]=1;queue.push(id);}};
 for(unsigned id=0;id<edges.size();id++)if(!edges[id].bend&&!edges[id].tether&&Length(Sub(x[edges[id].a],x[edges[id].b]))>edges[id].rest*limit*(1+1e-6))add(id);
 Result result;const unsigned budget=unsigned(edges.size())*64;
 while(!queue.empty()&&result.visits<budget){unsigned id=queue.front();queue.pop();scheduled[id]=0;result.visits++;auto e=edges[id];auto delta=Sub(x[e.a],x[e.b]);double distance=Length(delta),target=e.rest*limit,wa=inverseMass[e.a],wb=inverseMass[e.b];if(distance<=target*(1+1e-6)||distance<1e-20||wa+wb==0)continue;auto correction=Mul(delta,(distance-target)/(distance*(wa+wb)));if(wa){x[e.a]=Sub(x[e.a],Mul(correction,wa));for(auto neighbor:adjacency[e.a])add(neighbor);}if(wb){x[e.b]=Add(x[e.b],Mul(correction,wb));for(auto neighbor:adjacency[e.b])add(neighbor);}}
 result.exhausted=!queue.empty();return result;
}
// Sewing and extension share a mass-weighted constraint queue. Solving the
// seam afterwards would restore its endpoint and undo the fabric extension
// bound; every moved seam donor therefore reschedules its material neighbors.
template<class Edges,class Seams,class Residual> inline Result ProjectCoupled(std::vector<Point>&x,const std::vector<double>&inverseMass,const Edges&edges,const std::vector<std::vector<unsigned>>&adjacency,const Seams&seams,Residual residual,double limit){
 const unsigned edgeCount=unsigned(edges.size()),count=edgeCount+unsigned(seams.size());
 std::vector<std::vector<unsigned>> sewn(x.size());for(unsigned j=0;j<seams.size();j++)for(unsigned k=0;k<seams[j].count;k++)if(seams[j].weights[k])sewn[seams[j].nodes[k]].push_back(edgeCount+j);
 std::queue<unsigned> queue;std::vector<unsigned char> scheduled(count);auto add=[&](unsigned id){if(!scheduled[id]){scheduled[id]=1;queue.push(id);}};
 constexpr double relativeSolveTolerance=1e-3,seamSolveTolerance=1e-6;
 auto dirty=[&](unsigned node){for(auto id:adjacency[node]){const auto&e=edges[id];if(Length(Sub(x[e.a],x[e.b]))>e.rest*limit*(1+relativeSolveTolerance))add(id);}for(auto id:sewn[node]){const auto&s=seams[id-edgeCount];Point error=residual(s.residual);for(unsigned k=0;k<s.count;k++)error=Add(error,Mul(x[s.nodes[k]],s.weights[k]));if(Length(error)>=seamSolveTolerance)add(id);}};
 for(unsigned id=0;id<edgeCount;id++)if(!edges[id].bend&&!edges[id].tether&&Length(Sub(x[edges[id].a],x[edges[id].b]))>edges[id].rest*limit*(1+relativeSolveTolerance))add(id);
 for(unsigned j=0;j<seams.size();j++)add(edgeCount+j);
 Result result;const unsigned budget=count*128;
 while(!queue.empty()&&result.visits<budget){unsigned id=queue.front();queue.pop();scheduled[id]=0;result.visits++;
  if(id<edgeCount){auto e=edges[id];auto delta=Sub(x[e.a],x[e.b]);double distance=Length(delta),target=e.rest*limit,wa=inverseMass[e.a],wb=inverseMass[e.b];if(distance<=target*(1+relativeSolveTolerance)||distance<1e-20||wa+wb==0)continue;auto correction=Mul(delta,(distance-target)/(distance*(wa+wb)));if(wa){x[e.a]=Sub(x[e.a],Mul(correction,wa));dirty(e.a);}if(wb){x[e.b]=Add(x[e.b],Mul(correction,wb));dirty(e.b);}}
  else{const auto&s=seams[id-edgeCount];Point error=residual(s.residual);double sum=0;for(unsigned k=0;k<s.count;k++){error=Add(error,Mul(x[s.nodes[k]],s.weights[k]));sum+=inverseMass[s.nodes[k]]*s.weights[k]*s.weights[k];}if(sum<=1e-20||Length(error)<seamSolveTolerance)continue;for(unsigned k=0;k<s.count;k++)if(inverseMass[s.nodes[k]]&&s.weights[k]){x[s.nodes[k]]=Sub(x[s.nodes[k]],Mul(error,inverseMass[s.nodes[k]]*s.weights[k]/sum));dirty(s.nodes[k]);}}
 }
 result.exhausted=!queue.empty();return result;
}
}
