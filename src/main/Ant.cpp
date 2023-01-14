#include<random>
#include<cmath>
#include"Util.hpp"
#include"Ant.hpp"
#include"ACOTable.hpp"
#include"QuadTree.hpp"
#include"ACOSteiner.hpp"
using namespace std;

struct Joint{
  int route_index,index_in_route;
  double forward_ratio;
};

struct SingleRoute{
  Joint joint;
  vector<v2d> route;
  double cost=0,length=0;
};

Ant::Ant(const ACOSteiner &world) : BIRTH_TIME(world.getTime()),
                                    MIN_DISTANCE(world.getMinDistance()),
                                    MAX_DISTANCE(world.getMaxDistance()) {
  this->path.reserve(world.getConnectPoints().size());
  if(world.getACOTable().size())constructModifiedRoute(world,this->BIRTH_TIME);
  else constructFirstRoute(world);
  this->total_cost=this->total_length=0;
  for(int i=0;i<path.size();i++){
    this->total_cost+=this->path[i]->cost;
    this->total_length+=this->path[i]->length;
  }
  this->pheromone_v=this->total_cost;
}

inline const vector<v2d>& Ant::getRelayPointsOnRoute(int index) const {
  return this->path[index]->route;
}

inline int Ant::numOfPoints() const {return this->path.size();}
inline double Ant::cost() const {return this->total_cost;}
inline double Ant::length() const {return this->total_length;}
inline void Ant::evaporate(double evaporation_cofficient) const {this->pheromone_v*=evaporation_cofficient;}
inline double Ant::pheromone() const {return this->pheromone_v;}

//以下でprivate関数

static random_device seed_gen;
static default_random_engine random_engine(seed_gen());

void Ant::constructFirstRoute(const ACOSteiner &world){
  //TODO: 初期の蟻の実装!
}

void Ant::constructModifiedRoute(const ACOSteiner &world,ll current_time){
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

static double moveStandardDeviation(double distance_c2j,double basic_move_ratio,
                                    double stdev_cost,double mean_length,
                                    bool mutation){
  return distance_c2j*basic_move_ratio*min(1.0,10*stdev_cost/mean_length+mutation);
}

void Ant::addRandVecToOneRoute(const ACOSteiner &world, const Ant *base_ant,
                               const int target_index) {
  this->path[target_index].reset(new SingleRoute);
  const ACOTable &TABLE=world.getACOTable();
  const QuadTreeAnt &QTA=world.getQuadTreeAnt(target_index);
  const auto &START_POINT=world.getConnectPoint(target_index);
  const auto &BASE_ROUTE=base_ant->path[target_index]->route;
  auto &target_route=this->path[target_index]->route;
  QuadTree<const int> own_qt(QTA.width(),QTA.height());
  for(int i=0;i<this->path.size();i++)
    if(i!=target_index&&this->path[i]->joint.route_index!=target_index)
      own_qt.addRoute(this->path[i]->route,i);
  
  uniform_real_distribution<> uniform_unit_dist(0,1);
  //標準偏差を掛けると発散する危険があるので以下のようにする。
  const double move_standard_deviation=moveStandardDeviation(
    euclid(BASE_ROUTE[0],BASE_ROUTE[BASE_ROUTE.size()-1]),
    world.getBasicMoveRatio(),
    TABLE.stdev(ACOTableColumn::COST),
    TABLE.mean(ACOTableColumn::LENGTH),
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
  v2d min_point=QTA.minPoint(),max_point=min_point+QTA.size();
  for(int i=0;i<add_start_index;i++)
    properPushBack(target_route,BASE_ROUTE[i],min_point,max_point);
  //TODO: 接合点の移動を考慮する
  for(int i=add_start_index;i<add_main_target;i++){
    properPushBack(target_route,
        BASE_ROUTE[i]+base_random_vec*sin(M_PI/2*i/add_main_target),
        min_point,max_point);
  }
  for(int i=add_main_target;i<=actual_add_end_index;i++){
    properPushBack(
      target_route,
      BASE_ROUTE[i]+base_random_vec*sin(M_PI/2*(1+(double)i/original_add_end_index)),
      min_point,max_point
    );
  }
  for(int i=actual_add_end_index+1;i<BASE_ROUTE.size();i++)
    properPushBack(target_route,BASE_ROUTE[i],min_point,max_point);
}

void Ant::properPushBack(vector<v2d> &v,const v2d &e,const v2d &min_point,const v2d &max_point){
  v2d p{max(min_point[0],e[0]),max(min_point[1],e[1])};
  p[0]=min(p[0],max_point[0]);p[1]=min(p[1],max_point[1]);
  const double distance=euclid(v[v.size()-1],p);
  if(distance<MIN_DISTANCE)return;//NOTE: 簡単のため、近い時は重心を取るのではなくて追加しないようにした
  if(distance<=MAX_DISTANCE){//MIN_DISTANCE<=distanceは保証されている
    v.push_back(p);return;
  }
  const double ideal_complement_distance=mean(this->MIN_DISTANCE,this->MAX_DISTANCE);
  const int vector_num=ceil(distance/ideal_complement_distance);//NOTE: round等だと距離条件が満たされない場合がある
  for(int i=1;i<=vector_num;i++){
    v.push_back((double)i/vector_num*p);
  }
}
