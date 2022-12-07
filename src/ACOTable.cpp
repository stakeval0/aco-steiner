#include"ACOTable.hpp"

ACOTable::ACOTable(){
  //NOTE: 時間経過してもフェロモンの順序関係は変わらないのでこれで良い
  priority_queue([this](Ant &a,Ant &b){return a.pheromone(time)>b.pheromone(time);});
}
Ant ACOTable::selectAnt(){
  double sum=0;
  for(int i=0;i<this->c.size();i++)sum+=this->c[i].pheromone(this->time);
  //TODO: 実装!
}
double ACOTable::costVariance(){
  double sum=0,squareSum=0,temp;
  for(int i=0;i<this->c.size();i++){
    temp=this->c[i].allCost();
    sum+=temp;squareSum+=temp*temp;
  }
  return (squareSum-sum)/this->c.size();
}
void ACOTable::push(const Ant &a){
  priority_queue::push(a);
  while(this->size()>this->maxSize)this->pop();
}
void ACOTable::setTime(ll time){this->time=time;}
void ACOTable::setTableSize(ll size){this->maxSize=size;}