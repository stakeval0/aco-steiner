#include"Ant.hpp"
#include"ACOTable.hpp"

ACOTable::ACOTable(){
  //NOTE: 時間経過してもフェロモンの順序関係は変わらないのでこれで良い
  new (this) multiset([this](const Ant *a,const Ant *b){return a->pheromone(time)>b->pheromone(time);});
}

double ACOTable::costVariance(){
  double sum=0,square_sum=0,temp;
  for(const Ant *a:*this){
    temp=a->getAllCost();
    sum+=temp;square_sum+=temp*temp;
  }
  return (square_sum-sum)/this->size();
}

const Ant* ACOTable::dropout(){
  if(this->size()<=this->max_size)return nullptr;
  auto target=this->begin();
  const Ant* ret=*target;
  this->erase(target);
  return ret;
}

inline void ACOTable::setTime(ll time){this->time=time;}
inline void ACOTable::setTableSize(ll size){this->max_size=size;}