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
  acos.setMaxDistance(25);
  acos.setMinDistance(10);
  acos.setTableCapacity(30);
  acos.setBasicMoveRatio(1./100);
  acos.setEvaporationCofficient(0.95);
  cout<<acos.getEvaporationCofficient()<<endl;
  for(int i=0;i<100;i++){
    acos.search();
    for(auto &e:acos.getACOTable()){
      cout<<e->pheromone()<<','<<e->cost()<<' ';
    }
    cout<<endl;
    //const ACOTable &t=acos.getACOTable();
    //cout<<t.best(ACOTableColumn::PHEROMONE)<<','<<t.best(ACOTableColumn::COST)<<endl;
  }
  return 0;
}
