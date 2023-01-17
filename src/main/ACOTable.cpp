#include<cmath>
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

inline void ACOTable::setCapacity(ll size){this->max_size=size;}
inline ll ACOTable::getCapacity() const {return this->max_size;}

double ACOTable::sum(ACOTableColumn target) const {
  double (Ant::*getter)(void)const=findGetter(target);
  double ret=0;
  for(const Ant* a:*this)ret+=(a->*getter)();
  return ret;
}

inline double ACOTable::mean(ACOTableColumn target) const {
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

inline double ACOTable::stdev(ACOTableColumn target) const {
  return sqrt(this->variance(target));
}

inline json ACOTable::getJson() const {
  return getJson(0,this->size());
}

json ACOTable::getJson(const int begin, const int end) const {
  json ret=json::array();
  auto itr=this->begin();
  for(int i=0;i<=begin;i++)itr++;
  for(int i=begin;i<end&&itr!=this->end();i++,itr++){
    const Ant &a=**itr;
    ret.emplace_back(json::array());
    for(int j=0;j<a.routeNum();j++){
      ret[i].emplace_back(a.getRoute(j));
    }
  }
  return ret;
}

json ACOTable::getElementJson(const int index) const {
  json ret=json::array();
  auto itr=this->begin();
  for(int i=0;i<=index;i++)itr++;
  const Ant &a=**itr;
  for(int i=0;i<a.routeNum();i++){
    ret.emplace_back(a.getRoute(i));
  }
  return ret;
}
