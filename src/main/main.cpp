#include<iostream>
#include"Ant.hpp"
#include"ACOSteiner.hpp"
#include"ACOTable.hpp"
using namespace std;

int main(int argc, char const *argv[]){
  vector<array<double,2>> points={
    {0.,0.},
    {100.,0.},
    {0.,100.},
    {100.,100.}
  };
  ACOSteiner acos(points);
  acos.setMaxDistance(99);
  acos.setMinDistance(49);
  acos.setTableCapacity(100);
  acos.setBasicMoveRatio(0.01);
  acos.setEvaporationCofficient(0.9);
  acos.setMutationProbability(0.8);
  //cout<<acos.getEvaporationCofficient()<<endl;
  cout<<"[";
  //constexpr int n=10000;
  constexpr int n=10000;
  double min=__DBL_MAX__;
  for(int i=0;i<n;i++){
    acos.search();
    //for(auto &e:acos.getACOTable()){
    //  cout<<e->json()<<' ';
    //}
    //cout<<endl;
    //if(i<n-10)continue;
    const ACOTable &t=acos.getACOTable();
    if(min>t.best(ACOTableColumn::COST))min=t.best(ACOTableColumn::COST);
    //cout<<t.best(ACOTableColumn::PHEROMONE)<<','<<t.best(ACOTableColumn::COST)<<endl;
    const Ant &a=**t.begin();
    cout<<a.json();
    if(i<n-1)cout<<',';
  }
  cout<<']'<<endl;
  fprintf(stderr,"%g\n",min);
  return 0;
}
