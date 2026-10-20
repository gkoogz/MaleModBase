#include <malemod/garments/material_metric.hpp>
#include <iostream>
using namespace malemod::garments;
int main(){
 const std::vector<Point> uv{{0,0,0},{1,0,0},{0,1,0},{1,1,0}};
 const std::vector<std::array<unsigned,3>> faces{{0,1,2},{1,3,2}};
 // Fold the second triangle through 90 degrees about its shared diagonal.
 const double root=std::sqrt(.5);
 std::vector<Point> rest{{0,0,0},{1,0,0},{0,1,0},{.5,.5,root}};
 auto require=[](bool value){if(!value)throw std::runtime_error("Material metric regression");};
 require(std::abs(MaterialEdgeLength(uv,faces,rest,{0,0,0},{1,1,0})-std::sqrt(2.))<1e-12);
 require(std::abs(MaterialEdgeLength(uv,faces,rest,{1,0,0},{0,1,0})-std::sqrt(2.))<1e-12); // shared edge counted once
 require(std::abs(MaterialEdgeLength(uv,faces,rest,{1,1,0},{0,0,0})-std::sqrt(2.))<1e-12);
 for(auto& p:rest)p={20-3*p[1],-10+3*p[0],7+3*p[2]};
 require(std::abs(MaterialEdgeLength(uv,faces,rest,{0,0,0},{1,1,0})-3*std::sqrt(2.))<1e-12);
 bool rejected=false;try{MaterialEdgeLength(uv,faces,rest,{-1,0,0},{1,1,0});}catch(const std::invalid_argument&){rejected=true;}require(rejected);
 std::cout<<"Folded material metric, shared edge, reversal, transform and uncovered edge passed\n";
}
