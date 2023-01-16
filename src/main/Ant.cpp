#include<random>
#include<cmath>
#include<algorithm>
#include<stack>
#include"Util.hpp"
#include"Ant.hpp"
#include"ACOTable.hpp"
#include"QuadTree.hpp"
#include"ACOSteiner.hpp"
using namespace std;

struct Joint{
  int route_index=0,index_in_route;
  double forward_ratio;
};

struct SingleRoute{
  Joint joint;
  vector<v2d> points;
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
  return this->path[index]->points;
}

inline int Ant::numOfPoints() const {return this->path.size();}
inline double Ant::cost() const {return this->total_cost;}
inline double Ant::length() const {return this->total_length;}
inline void Ant::evaporate(double evaporation_cofficient) const {this->pheromone_v*=evaporation_cofficient;}
inline double Ant::pheromone() const {return this->pheromone_v;}

//以下でprivate関数

static random_device seed_gen;
static default_random_engine random_engine(seed_gen());
static uniform_real_distribution<> uniform_unit_dist(0,1);
static normal_distribution<> norm_unit_dist(0,1);

void Ant::constructFirstRoute(const ACOSteiner &world){
  //TODO: 初期の蟻の実装!
  const auto &connect_points=world.getConnectPoints();
  const auto &QTA0=world.getQuadTreeAnt(0);
  auto cost_lambda=[&world](const v2d &v1,const v2d &v2){
    return world.calcCost(v1,v2);
  };
  for(int i=0;i<this->path.size();i++)this->path[i].reset(new SingleRoute);
  this->path[0]->points.push_back(connect_points[0]);
  properPushBack(0,connect_points[1],cost_lambda,QTA0.minPoint(),QTA0.size());
}

static bool isDepend(const vector<shared_ptr<SingleRoute>> &path,
                       const uint parent,const uint child){
  uint ret;
  for(ret=child;ret&&ret!=parent;ret=path[ret]->joint.route_index);
  return ret==parent;
}

//独立なら-1、依存しているならその階数を保存するvector<int>を返す関数
vector<int> dependTree(const vector<shared_ptr<SingleRoute>> &path,
                       const uint parent){
  vector<int> ret(path.size(),-2);//未訪問なら-2
  ret[0]=-1;ret[parent]=0;
  for(int i=0;i<ret.size();i++){
    if(ret[i]!=-2)continue;
    stack<uint> s;
    uint tmp;
    for(tmp=i;ret[tmp]==-2;tmp=path[tmp]->joint.route_index){
      s.push(tmp);ret[tmp]=-1;
    }
    if(ret[tmp]<0)continue;
    for(uint count=ret[tmp]+1;!s.empty();count++){
      tmp=s.top();s.pop();
      ret[tmp]=count;
    }
  }
  return ret;
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
  const int target_route_index=random_engine()%this->path.size();
  if(this->MIN_DISTANCE==base_ant->MIN_DISTANCE&&this->MAX_DISTANCE==base_ant->MAX_DISTANCE){
    for(int i=0;i<this->path.size();i++)this->path[i]=base_ant->path[i];
  }else{
    for(int i=0;i<this->path.size();i++){
      //TODO: 間隔と接合点を考慮しながら経路をコピーするコードを書く
      __builtin_unreachable();//NOTE: 実装したら消す
    }
  }
  const auto &QTAV=world.getQuadTreeAntV();
  QuadTree<const int> own_qt(QTAV[0].minPoint(),QTAV[0].size());
  const vector<int> init_depend=dependTree(this->path,target_route_index);
  for(int i=0;i<this->path.size();i++)
    if(init_depend[i]<0)
      own_qt.addRoute(this->path[i]->points,i);

  auto [joined_target,add_range]=
      addRandVecToOneRoute(world,base_ant,target_route_index,own_qt);
  /*
    NOTE: 乱数を加えた範囲の接合点の移動修正をする。
          直接繋がっているものから順に接合していくのが良い。
          間接的に繋がっているものも、
          接合点とその直前の中継点の辺に繋がっている時は修正される。
  */
  //他の蟻と経路を接合させた時の接合点を処理するためには、ここで接合点の移動を考慮した方が都合が良い
  vector<int> &&after_depend=dependTree(this->path,target_route_index);
  vector<int> check_order(after_depend.size());
  for(int i=0;i<check_order.size();i++)check_order[i]=i;
  sort(check_order.begin(),check_order.end(),
       [&after_depend](const int &a,const int &b){
         return after_depend[a]<after_depend[b];
       });
  int own_order;
  for(own_order=0;
      own_order<check_order.size()&&after_depend[check_order[own_order]]<0;
      own_order++);//OPTIMIZE: ここを二分探索にしたい
  stack<uint> additional_decided_route;
  for(int i=own_order+1;i<check_order.size()==1;i++){
    const uint current_index=check_order[i];
    const Joint &joint=this->path[current_index]->joint;
    const auto &joint_points=this->path[check_order[joint.route_index]]->points;
    v2d joint_point=joint_points[joint.index_in_route];
    if(joint.index_in_route+1<joint_points.size()){
      const v2d forward_vec=joint_points[joint.index_in_route+1]-
                          joint_points[joint.index_in_route];
      joint_point+=joint.forward_ratio*forward_vec;
    }
    const auto &current_points=this->path[current_index]->points;
    if(euclid(joint_point,current_points[current_points.size()-1])<__DBL_EPSILON__){
      additional_decided_route.push(i);continue;
    }
    while(!additional_decided_route.empty()){
      uint tmp=additional_decided_route.top();additional_decided_route.pop();
      own_qt.addRoute(this->path[tmp]->points,tmp);
    }
    auto cost_lambda=[&world](const v2d &v1,const v2d &v2){
      return world.calcCost(v1,v2);
    };
    const Ant* tmp;
    for(int j=0;j<current_points.size();i++){
      tmp=joinToOwn(current_index,i,own_qt,world.getMaxDistance(),cost_lambda);
      if(tmp)break;
    }
    if(tmp)continue;
    const v2d &end_point=current_points[current_points.size()-1];
    v2d &nearest_point=this->path[0]->points[0];
    double min_distance=euclid(nearest_point,end_point);
    uint nearest_route_index,index_in_nearest_route;
    for(int j=0;j<i;j++){
      const auto &joint_target_points=this->path[check_order[j]]->points;
      for(int k=0;k<joint_target_points.size();k++){
        double distance=euclid(joint_target_points[k],end_point);
        if(min_distance>distance){
          min_distance=distance;
          nearest_point=joint_target_points[k];
          nearest_route_index=check_order[j];
          index_in_nearest_route=k;
        }
      }
    }
    QuadTreeNode<int> nearest_node{nearest_point,nearest_route_index,
                                   index_in_nearest_route};
    joinCloseToNearestNode(current_index,nearest_node,cost_lambda,own_qt);
  }
}

static double moveStandardDeviation(double distance_c2j,double basic_move_ratio,
                                    double stdev_cost,double mean_length,
                                    bool mutation){
  return distance_c2j*basic_move_ratio*min(1.0,10*stdev_cost/mean_length+mutation);
}

//NOTE: array<int,2>には乱数を加える開始と終了のインデックスが格納される
pair<const Ant*,const array<int,2>> Ant::addRandVecToOneRoute(
    const ACOSteiner &world, const Ant *base_ant,const int target_index,
    QuadTree<const int> &own_qt) {
  this->path[target_index].reset(new SingleRoute);
  const ACOTable &TABLE=world.getACOTable();
  const auto &BASE_ROUTE=base_ant->path[target_index]->points;
  
  //標準偏差を掛けると発散する危険があるので以下のようにする。
  const double move_standard_deviation=moveStandardDeviation(
    euclid(BASE_ROUTE[0],BASE_ROUTE[BASE_ROUTE.size()-1]),
    world.getBasicMoveRatio(),
    TABLE.stdev(ACOTableColumn::COST),
    TABLE.mean(ACOTableColumn::LENGTH),
    uniform_unit_dist(random_engine)>=world.getMutationProbability()||
      TABLE.size()<TABLE.getCapacity()
  );
  const double randvec_norm=norm_unit_dist(random_engine)*move_standard_deviation;
  array<double,2> base_random_vec{randvec_norm,0};
  rotate(base_random_vec,2*M_PI*uniform_unit_dist(random_engine));
  return archedAdd(target_index,base_ant,base_random_vec,world,own_qt);
}

pair<const Ant*,const array<int,2>> Ant::archedAdd(
    const int target_index,const Ant *base_ant,v2d base_random_vec,
    const ACOSteiner &world,QuadTree<const int> &own_qt){
  auto &target_points=this->path[target_index]->points,
       &base_points=base_ant->path[target_index]->points;
  const QuadTreeAnt &QTA=world.getQuadTreeAnt(target_index);
  
  const int add_main_target=random_engine()%base_points.size();
  const int arched_add_range=(random_engine()%(base_points.size()/2))*2+1;//奇数にしたい
  const int add_start_index=max(0,add_main_target-arched_add_range);
  const int original_add_end_index=add_main_target+arched_add_range/2;//NOTE: 本来の中継点の個数を上回ることもある
  const int actual_add_end_index=min((int)base_points.size()-1,original_add_end_index);
  
  target_points.reserve(base_points.size());
  const double rough_total_cost=base_ant->total_cost;
  v2d min_point=QTA.minPoint(),max_point=min_point+QTA.size();
  auto cost_lambda=[&world](const v2d &v1,const v2d &v2){
    return world.calcCost(v1,v2);
  };
  for(int i=0;i<add_start_index;i++)
    tracePushBack(target_index,base_points[i],cost_lambda,own_qt);
  //他の経路へ気づいて、短絡するならさせて接合した対象と、乱数を加えた範囲を返す
  int add_range=add_main_target-add_start_index;
  for(int i=add_start_index;i<add_main_target;i++){
    const int before_size=target_points.size();
    properPushBack(target_index,
        base_points[i]+base_random_vec*sin(M_PI/2*(i-add_start_index)/add_range),
        cost_lambda,own_qt);
    const Ant* tmp=judgeJoinTo(target_index,before_size,base_ant,QTA,own_qt,
                                world.getMaxDistance(),cost_lambda);
    if(tmp)return {tmp,{add_start_index,i}};
  }
  add_range=original_add_end_index-add_main_target;
  for(int i=add_main_target;i<=actual_add_end_index;i++){
    const int before_size=target_points.size();
    properPushBack(target_index,
        base_points[i]+base_random_vec*
            sin(M_PI/2*(1+(double)(i-add_main_target)/add_range)),
        cost_lambda,own_qt);
    const Ant* tmp=judgeJoinTo(target_index,before_size,base_ant,QTA,own_qt,
                                world.getMaxDistance(),cost_lambda);
    if(tmp)return {tmp,{add_start_index,i}};
  }
  for(int i=actual_add_end_index+1;i<base_points.size();i++)
    tracePushBack(target_index,base_points[i],cost_lambda,own_qt);
  return {nullptr,{add_start_index,actual_add_end_index}};
}

void Ant::tracePushBack(
    const int target_index,const v2d &e,
    const function<double(const v2d&,const v2d&)> &cost_function,
    QuadTree<const int> &own_qt){
  SingleRoute &route=*(this->path[target_index]);
  auto &points=route.points;
  const v2d &last_point=points[points.size()-1];
  own_qt.addPoint(e,points.size(),target_index);
  points.push_back(e);
  route.cost+=cost_function(last_point,e);
  route.length+=euclid(last_point,e);
}

void Ant::properPushBack(
    const int target_index,const v2d &e,
    const function<double(const v2d&,const v2d&)> &cost_function,
    const v2d &min_point,const v2d &size){
  const v2d &max_point=min_point+size;
  SingleRoute &route=*(this->path[target_index]);
  auto &points=route.points;
  const v2d &last_point=points[points.size()-1];
  v2d p{max(min_point[0],e[0]),max(min_point[1],e[1])};
  p[0]=min(p[0],max_point[0]);p[1]=min(p[1],max_point[1]);
  const double distance=euclid(last_point,p);
  if(distance<this->MIN_DISTANCE)return;//NOTE: 簡単のため、近い時は重心を取るのではなくて追加しないようにした
  if(distance<=this->MAX_DISTANCE){//this->MIN_DISTANCE<=distanceは保証されている
    points.push_back(p);
  }else{
    const double ideal_complement_distance=mean(this->MIN_DISTANCE,this->MAX_DISTANCE);
    const int vector_num=ceil(distance/ideal_complement_distance);//NOTE: round等だと距離条件が満たされない場合がある
    for(int i=1;i<=vector_num;i++){
      points.push_back((double)i/vector_num*p);
    }
  }
  //TODO: 自身の経路が交差していたら短絡させる
  route.cost+=cost_function(last_point,p);
  route.length+=distance;
}

void Ant::properPushBack(
    const int target_index,const v2d &e,
    const function<double(const v2d&,const v2d&)> &cost_function,
    QuadTree<const int> &own_qt){
  const auto &points=this->path[target_index]->points;
  const uint before_size=points.size();
  properPushBack(target_index,e,cost_function,own_qt.minPoint(),own_qt.size());
  const uint after_size=points.size();
  for(int i=before_size;i<after_size;i++)
    own_qt.addPoint(points[i],i,target_index);
}

inline static bool circleFilter(const v2d &v,const v2d &c,double radius){
  return euclid(v-c)<=radius;
}

const Ant* Ant::judgeJoinTo(
    const int route_index,const int first_index_in_route,const Ant* base_ant,
    const QuadTree<const Ant*> &qta,QuadTree<const int> &own_qt,
    const double reachable_radius,
    const function<double(const v2d&,const v2d&)> &cost_function){
  const vector<v2d> &current_points=this->path[route_index]->points;
  const v2d &current_point=current_points[current_points.size()-1];
  const double allowance_reachable_radius=reachable_radius+__DBL_EPSILON__;
  const double allowance_reachable_diameter=2*allowance_reachable_radius;
  const auto filter=[&current_point,allowance_reachable_radius](const v2d &p){
    return circleFilter(p,current_point,allowance_reachable_radius);
  };
  for(int i=first_index_in_route;i<current_points.size();i++){
    const auto own_join_result=
        joinToOwn(route_index,i,own_qt,reachable_radius,cost_function);
    if(own_join_result)return own_join_result;
  }
  /*
    TODO: ant_reachablesはmapを使って、蟻毎に最も接合点に近い点を候補にする
    NOTE: 接合の処理もここで行う。つまり、own_qtやthis->pathの付け替えなども行う。
          上の階層に投げるのは、あくまで乱数を加えた範囲の接合点の移動修正のみである。
  */
  //const auto &ant_reachables=qta.reachablePoints(
  //    current_point[0],current_point[1],allowance_reachable_diameter,
  //    allowance_reachable_diameter,filter);
  return nullptr;//どちらにも接合しない時に返す
}

//FIXME: 最近傍辺による方式に変更する
const Ant* Ant::joinToOwn(
    const int route_index,const int index_in_route,QuadTree<const int> &own_qt,
    const double reachable_radius,
    const function<double(const v2d&,const v2d&)> &cost_function){
  vector<v2d> &current_points=this->path[route_index]->points;
  const v2d &current_point=current_points[index_in_route];
  const double allowance_reachable_radius=reachable_radius+__DBL_EPSILON__;
  const double allowance_reachable_diameter=2*allowance_reachable_radius;
  const auto filter=[&current_point,allowance_reachable_radius](const v2d &p){
    return circleFilter(p,current_point,allowance_reachable_radius);
  };
  vector<QuadTreeNode<const int>> &&points_reachables=own_qt.reachablePoints(
      current_point[0],current_point[1],allowance_reachable_diameter,
      allowance_reachable_diameter,filter);
  if(points_reachables.size()==0)return nullptr;
  //NOTE: 以下で乱数を加えたところが自分の既に確定している経路と衝突していないことを保証
  const QuadTreeNode<const int> *tmp=&points_reachables[0];
  double min_distance,tmp_distance;
  for(int i=1;i<points_reachables.size();i++){
    if(tmp->value!=route_index)continue;
    tmp_distance=euclid(tmp->point,current_point);
    if(tmp_distance<min_distance){
      min_distance=tmp_distance;
      tmp=&points_reachables[i];
    }
  }
  //最も近い中継点が関係する辺の内、最も近いところにくっつける
  const auto &nearest_node=*tmp;
  //NOTE: 上で&points_reachables[0]から変わらなかった場合などは弾かなければならない
  if(euclid(nearest_node.point,current_point)>allowance_reachable_radius)
    return nullptr;
  current_points.resize(index_in_route+1);
  joinCloseToNearestNode(route_index,nearest_node,cost_function,own_qt);
  return this;
}

static bool isCross(const v2d &current_point,const v2d &last_point,
                    const v2d &nearest_point,const v2d &other_side_point){
  const v2d l2c_vec=current_point-last_point,
            l2n_vec=nearest_point-last_point,
            l2o_vec=other_side_point-last_point;
  return cross(l2c_vec,l2n_vec)*cross(l2c_vec,l2o_vec)<=0;
}

template<class T>
void Ant::joinCloseToNearestNode(
    const int target_index,const QuadTreeNode<T> &nearest_node,
    const function<double(const v2d&,const v2d&)> &cost_function,
    QuadTree<const int> &own_qt){
  const vector<v2d> &current_points=this->path[target_index]->points;
  const v2d &current_point=current_points[current_points.size()-1];
  const v2d nearest2current_vec=current_point-nearest_node.point;
  const double nearest2current_norm_2=pow(euclid(nearest2current_vec),2);
  const auto &joint_target_route=*(this->path[nearest_node.value]);
  v2d back_nearest,forward_nearest;
  back_nearest=forward_nearest=nearest_node.point;
  double back_ratio=0,forward_ratio=0;
  if(nearest_node.index>0){
    const v2d back_vector=
        joint_target_route.points[nearest_node.index-1]-nearest_node.point;
    const v2d &ortho_vec=dot(back_vector,nearest2current_vec)/
                             nearest2current_norm_2*back_vector;
    back_ratio=max(0.,dot(ortho_vec,back_vector));
    if(back_ratio>0)back_nearest=nearest_node.point+ortho_vec;
  }
  if(nearest_node.index+1<joint_target_route.points.size()){
    const v2d forward_vector=
        joint_target_route.points[nearest_node.index+1]-nearest_node.point;
    const v2d &ortho_vec=dot(forward_vector,nearest2current_vec)/
                             nearest2current_norm_2*forward_vector;
    forward_ratio=max(0.,dot(ortho_vec,forward_vector));
    if(forward_ratio>0)forward_nearest=nearest_node.point+ortho_vec;
  }
  Joint &target_joint=this->path[target_index]->joint;
  target_joint.route_index=nearest_node.value;
  if(cost_function(forward_nearest,current_point)<=
        cost_function(back_nearest,current_point)){
    properPushBack(target_index,forward_nearest,cost_function,own_qt);
    target_joint.index_in_route=nearest_node.index;
    target_joint.forward_ratio=forward_ratio;
  }else{
    properPushBack(target_index,back_nearest,cost_function,own_qt);
    target_joint.index_in_route=nearest_node.index-1;
    target_joint.forward_ratio=1-forward_ratio;
  }
}
