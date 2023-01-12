#include<random>
#include<cmath>
#include"Util.hpp"
#include"Ant.hpp"
#include"ACOSteiner.hpp"
#include"ACOTable.hpp"
using namespace std;

Ant::Ant(const ACOSteiner &world) : BIRTH_TIME(world.getTime()),
                                    MIN_DISTANCE(world.getMinDistance()),
                                    MAX_DISTANCE(world.getMaxDistance()) {
  this->path.reserve(world.getConnectPoints().size());
  if(world.getACOTable().size())searchNewRoute(world,this->BIRTH_TIME);
  else constructFirstAnt(world);
}

//inline tuple<int,int,double> Ant::getBackTimesOnRoute(int index) const {
//  return this->path[index].first;
//}

inline const vector<v2d>& Ant::getRelayPointsOnRoute(int index) const {
  return *(this->path[index].second);
}

const pair<tuple<int,int,double>,const vector<v2d> &> Ant::getRoute(int index) const {
  return {this->path[index].first,*(this->path[index].second)};
}

inline int Ant::numOfPoints() const {return this->path.size();}
inline double Ant::allCost() const {return this->all_cost;}
inline void Ant::evaporate(double evaporation_cofficient) const {this->pheromone_v*=evaporation_cofficient;}
inline double Ant::pheromone() const {return this->pheromone_v;}

//以下でprivate関数

static random_device seed_gen;
static default_random_engine random_engine(seed_gen());

void Ant::constructFirstAnt(const ACOSteiner &world){
  //TODO: 初期の蟻の実装!
}

void Ant::searchNewRoute(const ACOSteiner &world,ll current_time){
  const ACOTable &TABLE=world.getACOTable();
  double pheromone_sum=TABLE.sum(ACOTableColumn::PHEROMONE);
  uniform_real_distribution<> dist(0,pheromone_sum);
  double rand=dist(random_engine),sum=0;
  const Ant *base_ant;
  for(const auto &e:TABLE){
    sum+=e->pheromone();
    if(sum>=rand){base_ant=e;break;}
  }
  if(this->MIN_DISTANCE==base_ant->MIN_DISTANCE&&this->MAX_DISTANCE==base_ant->MAX_DISTANCE){
    for(int i=0;i<this->path.size();i++)this->path[i]=base_ant->path[i];
  }else{
    for(int i=0;i<this->path.size();i++){
      //TODO: 間隔と接合点を考慮しながら経路をコピーするコードを書く
    }
  }
  int target_route_index=random_engine()%this->path.size();
  addRandVecToOneRoute(world,base_ant,target_route_index);
}

static double moveStandardDeviation(double distance_mean,double cost_std_deviation,bool mutation){
  static const double EXPONENT_BASE=pow(0.5,20);
  return distance_mean/10*(1-pow(EXPONENT_BASE,cost_std_deviation/distance_mean)+mutation);
}

void Ant::addRandVecToOneRoute(const ACOSteiner &world, const Ant *base_ant,
                               const int index) {
  this->path[index].second.reset(new vector<v2d>);
  const ACOTable &TABLE=world.getACOTable();
  const QuadTreeAnt &QT=world.getQuadTreeAnt(index);
  const auto &START_POINT=world.getConnectPoint(index);
  const auto &BASE_ROUTE=*(base_ant->path[index].second);
  auto &target_route=*(this->path[index].second);
  uniform_real_distribution<> uniform_unit_dist(0,1);
  //標準偏差を掛けると発散する危険があるので以下のようにする。
  const double move_standard_deviation=moveStandardDeviation(
    TABLE.mean(ACOTableColumn::COST),
    TABLE.stdev(ACOTableColumn::COST),
    uniform_unit_dist(random_engine)>=world.getMutationProbability()||
      TABLE.size()<TABLE.getCapacity()
  );
  normal_distribution<> dist_norm(0,move_standard_deviation);
  const double randvec_norm=dist_norm(random_engine);
  array<double,2> base_random_vec{randvec_norm,0};
  rotate(base_random_vec,2*M_PI*uniform_unit_dist(random_engine));
  const int add_main_target=random_engine()%target_route.size();
  const int arched_add_range=(random_engine()%(target_route.size()/2))*2+1;//奇数にしたい
  const int add_start_index=max(0,add_main_target-arched_add_range);
  const int original_add_end_index=add_main_target+arched_add_range/2;//NOTE: 本来の中継点の個数を上回ることもある
  const int actual_add_end_index=min((int)BASE_ROUTE.size()-1,original_add_end_index);
  target_route.reserve(BASE_ROUTE.size());
  vector<int> b2t_index_map(BASE_ROUTE.size());
  for(int i=0;i<add_start_index;i++){
    properPushBack(target_route,BASE_ROUTE[i]);
    b2t_index_map[i]=i;
  }
  //TODO: 接合点の移動を考慮する
  for(int i=add_start_index;i<add_main_target;i++){
    properPushBack(target_route,BASE_ROUTE[i]+base_random_vec*sin(M_PI/2*i/add_main_target));
    b2t_index_map[i]=target_route.size()-1;
  }
  for(int i=add_main_target;i<=actual_add_end_index;i++){
    properPushBack(
      target_route,
      BASE_ROUTE[i]+base_random_vec*sin(M_PI/2*(1+(double)i/original_add_end_index))
    );
    b2t_index_map[i]=target_route.size()-1;
  }
  for(int i=actual_add_end_index+1;i<BASE_ROUTE.size();i++){
    properPushBack(target_route,BASE_ROUTE[i]);
    b2t_index_map[i]=i;
  }
}

void Ant::properPushBack(vector<v2d> &v,const v2d &e){
  const double distance=euclid(v[v.size()-1],e);
  if(distance<MIN_DISTANCE)return;//NOTE: 簡単のため、近い時は重心を取るのではなくて追加しないようにした
  if(distance<=MAX_DISTANCE){//MIN_DISTANCE<=distanceは保証されている
    v.push_back(e);return;
  }
  const double ideal_complement_distance=mean(this->MIN_DISTANCE,this->MAX_DISTANCE);
  const int vector_num=ceil(distance/ideal_complement_distance);//NOTE: round等だと距離条件が満たされない場合がある
  for(int i=1;i<=vector_num;i++){
    v.push_back((double)i/vector_num*e);
  }
}
