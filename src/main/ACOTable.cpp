#include<cmath>
#include<sstream>
#include"Ant.hpp"
#include"ACOTable.hpp"
using ll=long long;

//引数にACOTableColumnを取るfindGetterという関数で、Antクラスの、引数がvoid、返値がdoubleな、フィールドを変えない関数(const)の関数ポインタを返す
static double (Ant::*findGetter(ACOTableColumn col))() const {
  switch (col){
    case ACOTableColumn::PHEROMONE:
      return &Ant::pheromone;
    case ACOTableColumn::COST:
      return &Ant::cost;
    case ACOTableColumn::LENGTH:
      return &Ant::length;
  }
  __builtin_unreachable();
}

ACOTable::ACOTable(){
  //NOTE: 時間経過してもフェロモンの順序関係は変わらないのでこれで良い
  new (this) multiset([this](const Ant *a,const Ant *b){return a->pheromone()>b->pheromone();});
}

const Ant* ACOTable::dropout(){
  if(this->size()<=this->max_size)return nullptr;
  auto target=this->begin();
  const Ant* ret=*target;
  this->erase(target);
  return ret;
}

double ACOTable::sum(ACOTableColumn target) const {
  double (Ant::*getter)(void)const=findGetter(target);
  double ret=0;
  for(const Ant* a:*this)ret+=(a->*getter)();
  return ret;
}

double ACOTable::mean(ACOTableColumn target) const {
  return this->sum(target)/this->size();
}

double ACOTable::variance(ACOTableColumn target) const {
  double (Ant::*getter)(void)const=findGetter(target);
  double sum=0,square_sum=0,temp;
  for(const Ant *a:*this){
    temp=(a->*getter)();
    sum+=temp;square_sum+=temp*temp;
  }
  double mean=sum/this->size();
  return square_sum/this->size()-mean*mean;
}

double ACOTable::stdev(ACOTableColumn target) const {
  return sqrt(this->variance(target));
}

double ACOTable::best(ACOTableColumn target) const {
  double (Ant::*getter)(void)const=findGetter(target);
  return ((*(this->begin()))->*getter)();
}

string ACOTable::json() const {
  return this->json(0,this->size());
}

string ACOTable::json(const int begin, const int end) const {
  stringstream ss;
  ss<<'[';
  auto itr=this->begin();
  for(int i=0;i<=begin;i++)itr++;
  for(int i=begin;i<end&&itr!=this->end();i++,itr++){
    const Ant &a=**itr;
    ss<<a.json()<<',';
  }
  ss<<']';
  return ss.str();
}
