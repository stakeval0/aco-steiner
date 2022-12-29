#include<random>
#include"Ant.hpp"
#include"ACOSteiner.hpp"
#include"ACOTable.hpp"
using namespace std;

static random_device seed_gen;
static default_random_engine random_engine(seed_gen());

void Ant::constructOneRoute(const ACOSteiner &world,const Ant *base_ant,int index){
  this->path[index].second.reset(new vector<array<double,2>>);
  const ACOTable &TABLE=world.getACOTable();
  const QuadTree &QT=world.getQuadTree(index);
  const auto &BASE_ROUTE=*(base_ant->path[index].second);
  auto &target_route=*(this->path[index].second);
  target_route.reserve(BASE_ROUTE.size());
  double move_standard_deviation;//TODO: 実装! ここでもeuclidが欲しくなる
  //normal_distribution<> dist(0,)
  //TODO: 戻る回数を元経路の乱数の追加して保存
}

void Ant::searchNewRoute(const ACOSteiner &world,ll current_time){
  const ACOTable &TABLE=world.getACOTable();
  double pheromone_sum=0;
  //NOTE: ここのループで累積和とイテレーターを詰めた配列を構成すれば経路選択では二分探索が使える
  for(const auto &e:TABLE)pheromone_sum+=e->pheromone(current_time);
  uniform_real_distribution<> dist(0,pheromone_sum);
  double rand=dist(random_engine),sum=0;
  const Ant *base_ant;
  for(const auto &e:TABLE){
    sum+=e->pheromone(this->birth_time);
    if(sum>=rand){base_ant=e;break;}
  }
  //FIXME: 以下のコードを、1つの経路のみconstructOneRouteし、その他の経路はbase_antからコピーするように変える
  for(int i=0;i<this->path.size();i++){
    constructOneRoute(world,base_ant,i);
  }
}

Ant::Ant(const ACOSteiner &world,bool init){
  this->birth_time=world.getTime();
  if(!init)searchNewRoute(world,this->birth_time);
}

inline int Ant::getBackTimesOnRoute(int index) const {
  return this->path[index].first;
}

inline const vector<array<double,2>>& Ant::getRelayPointsOnRoute(int index) const {
  return *(this->path[index].second);
}

const tuple<int,const vector<array<double,2>> &> Ant::getRoute(int index) const {
  return forward_as_tuple(this->path[index].first,*(this->path[index].second));
}

inline int Ant::numOfPoints() const {return this->path.size();}

/* Ant* Ant::searchNewRoute(ACOSteiner world){
   Ant *ret=new Ant();
 } */