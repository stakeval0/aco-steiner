#include<iostream>
#include<vector>
#include<array>
#include<random>
#include"../main/QuadTree.hpp"
using namespace std;

int main(void){
  QuadTree qt=QuadTree(100.0,100.0);
  vector<vector<array<double,2>>>routes(100,vector<array<double,2>>(100));
  random_device seed_gen;
  default_random_engine engine(seed_gen());
  uniform_real_distribution<> dist(0,100);
  for(int i=0;i<100;i++){
    for(int j=0;j<100;j++){
      for(int k=0;k<routes[i][j].size();k++){
        double tmp=dist(engine);
        routes[i][j][k]=tmp;
        cout<<tmp<<(k<routes[i][j].size()-1?',':'\n');
      }
    }
    qt.addRoute(routes[i],NULL);
  }
  auto nodes=qt.reachablePoints(50,50,25,25);
  for(int i=0;i<nodes->size();i++){
    const array<double,2> &p=(*nodes)[i].first;
    printf("%g,%g\n",p[0],p[1]);
  }
  delete(nodes);
  //cout<<(uint)(UINT32_MAX>>32)<<endl;
  return 0;
}