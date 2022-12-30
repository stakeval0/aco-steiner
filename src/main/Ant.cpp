#include<random>
#include<cmath>
#include"Util.hpp"
#include"Ant.hpp"
#include"ACOSteiner.hpp"
#include"ACOTable.hpp"
using namespace std;

static random_device seed_gen;
static default_random_engine random_engine(seed_gen());

Ant::Ant(const ACOSteiner &world) : BIRTH_TIME(world.getTime()) {
  this->path.reserve(world.getConnectPoints().size());
  if(world.getACOTable().size())searchNewRoute(world,this->BIRTH_TIME);
  else constructFirstAnt(world);
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
inline double Ant::allCost() const {return this->all_cost;}
inline void Ant::evaporate(double evaporation_cofficient) const {this->pheromone_v*=evaporation_cofficient;}
inline double Ant::pheromone() const {return this->pheromone_v;}

//以下でprivate関数

void Ant::constructFirstAnt(const ACOSteiner &world){
  //TODO: 初期の蟻の実装!
}

void Ant::searchNewRoute(const ACOSteiner &world,ll current_time){
  const ACOTable &TABLE=world.getACOTable();
  double pheromone_sum=0;
  //NOTE: ここのループで累積和とイテレーターを詰めた配列を構成すれば経路選択では二分探索が使える
  for(const auto &e:TABLE)pheromone_sum+=e->pheromone();
  uniform_real_distribution<> dist(0,pheromone_sum);
  double rand=dist(random_engine),sum=0;
  const Ant *base_ant;
  for(const auto &e:TABLE){
    sum+=e->pheromone();
    if(sum>=rand){base_ant=e;break;}
  }
  //NOTE: 以下のコードでは中継点同士の距離のパラメーターの変更を考慮していないが、どうせ間を線形補完するだけなのでこれで良い
  int target_route_index=random_engine()%this->path.size();
  for(int i=0;i<this->path.size();i++)this->path[i]=base_ant->path[i];
  addRandVecToOneRoute(world,base_ant,target_route_index);
}

void Ant::addRandVecToOneRoute(const ACOSteiner &world,const Ant *base_ant,const int index){
  this->path[index].second.reset(new vector<array<double,2>>);
  const ACOTable &TABLE=world.getACOTable();
  const QuadTree &QT=world.getQuadTree(index);
  const auto &START_POINT=world.getConnectPoint(index);
  const auto &BASE_ROUTE=*(base_ant->path[index].second);
  auto &target_route=*(this->path[index].second);
  target_route.reserve(BASE_ROUTE.size());
  /* double move_standard_deviation=
      base_ant->allCost() * world.getBasicMoveRatio() *
      (TABLE.costVariance() +
      (random_engine()>=world.getMutationProbability())||TABLE.size()<TABLE.getCapacity()); */
  //標準偏差を掛けると発散する危険があるので以下のようにする。
  double move_standard_deviation=base_ant->allCost()*world.getBasicMoveRatio();
  normal_distribution<> dist_norm(0,move_standard_deviation);
  uniform_real_distribution<> dist_theta(0,2*M_PI);
  double randvec_norm=dist_norm(random_engine);
  array<double,2> base_random_vec{randvec_norm,0};
  rotate(base_random_vec,dist_theta(random_engine));
  int add_main_target=random_engine()%target_route.size();
  int arched_add_range=random_engine()%target_route.size();
  this->path[index].second->reserve(target_route.size());
  const auto *current_relay_point=&START_POINT,
             *next_relay_point=&target_route[0];
  for(int i=0;i<target_route.size();i++){
    /*
      間を補完しながら乱数を加えていきたい。
    */
  }
  //TODO: 実装!
}
