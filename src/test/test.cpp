#include<iostream>
#include<vector>
#include<array>
#include<random>
#include<set>
#include<stack>
#include"../tools/json.hpp"
//#include"../main/QuadTree.hpp"
using namespace std;

vector<int> dependTree(const vector<int> &path,
                       const uint parent){
  vector<int> ret(path.size(),-2);//未訪問なら-2
  ret[0]=-1;ret[parent]=0;
  for(int i=0;i<ret.size();i++){
    if(ret[i]!=-2)continue;
    stack<uint> s;
    uint tmp;
    for(tmp=i;ret[tmp]==-2;tmp=path[tmp]){
      s.push(tmp);ret[tmp]=-1;
    }
    if(ret[tmp]<0)continue;
    for(uint count=ret[tmp]+1;!s.empty();count++){
      tmp=s.top();s.pop();
      ret[tmp]=count;
    }
  }
  return ret;
}

void dependTreeTest(){
  vector<int> hoge{0,3,0,0,1};
  const vector<int> &fuga=dependTree(hoge,1);
  for(auto &e:fuga){
    cout<<e<<' ';
  }
  cout<<endl;
}

//void forStaticTest(int n){
//  for(int i=0;i<n;i++){
//    static int hoge=n;
//    if(i==n-1)cout<<hoge<<endl;
//  }
//}

//void setTest(){
//  set<pair<int,const array<int,2>*>> s;
//  const array<int,2>hoge={1,2},fuga={1,2};
//  s.insert({1,&hoge});
//  s.erase({1,&fuga});
//  cout<<(s.empty()?"empty":"not empty")<<endl;
//}

/*
void quadTreeTest(){
  QuadTree<const int> qt({10,10},{100.0,100.0});
  vector<vector<array<double,2>>>routes(100,vector<array<double,2>>(100));
  random_device seed_gen;
  default_random_engine engine(seed_gen());
  uniform_real_distribution<> dist(10,110);
  for(int i=0;i<100;i++){
    for(int j=0;j<100;j++){
      for(int k=0;k<routes[i][j].size();k++){
        double tmp=dist(engine);
        routes[i][j][k]=tmp;
        cout<<tmp<<(k<routes[i][j].size()-1?',':'\n');
      }
    }
    qt.addRoute(routes[i],0);
  }
  auto nodes=qt.reachablePoints(50,50,50,50);
  for(int i=0;i<nodes.size();i++){
    const array<double,2> &p=nodes[i].point;
    printf("%g,%g\n",p[0],p[1]);
  }
}
*/

using json=nlohmann::json;
void jsonTest(){
  vector<array<double,2>> data(5);
  for(int i=0;i<data.size();i++){
    data[i][0]=2*i;data[i][1]=data[i][0]+1;
  }
  json j=json::array();
  j.emplace_back(data);
  j.emplace_back(data);
  cout<<j<<endl;
}

int main(void){
  //forStaticTest(1);
  //forStaticTest(100);
  //quadTreeTest();
  //cout<<(uint)(UINT32_MAX>>32)<<endl;
  //dependTreeTest();
  jsonTest();
  return 0;
}