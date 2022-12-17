#include"ACOTable.hpp"

ACOTable::ACOTable(){
  //NOTE: 時間経過してもフェロモンの順序関係は変わらないのでこれで良い
  multiset([this](Ant *a,Ant *b){return a->pheromone(time)>b->pheromone(time);});
}
Ant ACOTable::selectAnt(){
  double sum=0;
  //TODO: 実装!
}
double ACOTable::costVariance(){
  double sum=0,squareSum=0,temp;
  for(Ant *a:*this){
    temp=a->getAllCost();
    sum+=temp;squareSum+=temp*temp;
  }
  return (squareSum-sum)/this->size();
}
void ACOTable::insert(Ant *a){
  multiset::insert(a);
  if(this->size()>this->maxSize)this->erase(this->begin());
}
void ACOTable::setTime(ll time){this->time=time;}
void ACOTable::setTableSize(ll size){this->maxSize=size;}