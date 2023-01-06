#include<iostream>
#include<cmath>
#include"Util.hpp"
#include"Ant.hpp"
#include"ACOSteiner.hpp"
#include"ACOTable.hpp"
#include"QuadTree.hpp"

constexpr double MIN_SPACE_RATIO_DEFAULT=1.0/100,MAX_SPACE_RATIO_DEFAULT=1.0/10;

static inline void printArgumentErrorMessage(const char *func_name,const char *correct_argument_condition){
  fprintf(stderr,"%s: argument needs to be %s.\n",func_name,correct_argument_condition);
}

ACOSteiner::ACOSteiner(const vector<array<double,2>> &points){
  this->table=new ACOTable();
  this->cost_function=euclid;
  double xmin=0,xmax=0,ymin=0,ymax=0;
  for(int i=0;i<points.size();i++){
    xmin=min(xmin,points[i][0]);xmax=max(xmax,points[i][0]);
    ymin=min(ymin,points[i][1]);ymax=max(ymax,points[i][1]);
  }
  this->points_offset={xmin,ymin};
  this->pheromone_cofficient=0;
  for(int i=0;i<points.size();i++){
    this->points[i][0]=points[i][0]-xmin;
    this->points[i][1]=points[i][1]-ymin;
  }
  double width=xmax-xmin,height=ymax-ymin;
  int num_of_route=points.size()-1;
  this->qtworld.reserve(num_of_route);
  for(int i=0;i<num_of_route;i++){
    this->qtworld.emplace_back(width,height);
    this->pheromone_cofficient=cost_function(points[i],points[i+1]);
  }
  double diagnoal=sqrt(width*width+height*height);
  this->min_distance=diagnoal*MIN_SPACE_RATIO_DEFAULT;
  this->max_distance=diagnoal*MAX_SPACE_RATIO_DEFAULT;
}

ACOSteiner::~ACOSteiner(){
  delete this->table;//ACOTableをunique_ptrにすれば書かなくて良くなる
}

void ACOSteiner::search(){
  countTime();
  Ant *new_ant=new Ant(*this);
  this->table->insert(new_ant);
  //4分木の世界へ登録
  for(int i=0;i<new_ant->numOfPoints();i++){
    this->qtworld[i].addRoute(new_ant->getRelayPointsOnRoute(i),new_ant);
  }
  //最悪解の削除
  const Ant *dropout_ant=this->table->dropout();
  if(dropout_ant){
    for(int i=0;i<dropout_ant->numOfPoints();i++){
      this->qtworld[i].removeRoute(dropout_ant->getRelayPointsOnRoute(i),dropout_ant);
    }
    //設計段階でここでdeleteすれば必要十分である事が証明されているが、後でスマートポインタにしても良いかも知れない
    delete dropout_ant;
  }
}

inline void ACOSteiner::setCostFunction(const function<double(const array<double,2> &,const array<double,2> &)> &f){
  this->cost_function=f;
}

inline double ACOSteiner::calcCost(const array<double,2> &p1,const array<double,2> &p2) const {return this->cost_function(p1,p2);}

void ACOSteiner::setMinDistance(double d){
  if(d>=0&&d<=this->max_distance/2){
    this->min_distance=d;return;
  }
  printArgumentErrorMessage(__PRETTY_FUNCTION__,"no less than 0 and no more than half max_distance");
}

void ACOSteiner::setMaxDistance(double d){
  if(d>0&&d>=this->min_distance*2){
    this->max_distance=d;return;
  }
  printArgumentErrorMessage(__PRETTY_FUNCTION__,"more than 0 and no less than twice min_distance");
}

void ACOSteiner::setMutationProbability(double value){
  if(value>=0&&value<=1){
    this->mutation_probability=value;return;
  }
  printArgumentErrorMessage(__PRETTY_FUNCTION__,"value from 0.0 to 1.0");
}

void ACOSteiner::setEvaporationCofficient(double value){
  if(value>=0&&value<=1){
    this->evaporation_cofficient=value;return;
  }
  printArgumentErrorMessage(__PRETTY_FUNCTION__,"value from 0.0 to 1.0");
}

void ACOSteiner::setBasicMoveRatio(double value){
  if(value!=0){
    this->basic_move_ratio=value;return;
  }
  printArgumentErrorMessage(__PRETTY_FUNCTION__,"not 0");
}

inline void ACOSteiner::setTableCapacity(ll size){this->table->setCapacity(size);}

inline ll ACOSteiner::getTime() const {return this->time;}
inline const array<double,2>& ACOSteiner::getConnectPoint(int index) const {
  return this->points[index];
}
inline const vector<array<double,2>>& ACOSteiner::getConnectPoints() const {
  return this->points;
}
inline double ACOSteiner::getMinDistance() const {return this->min_distance;}
inline double ACOSteiner::getMaxDistance() const {return this->max_distance;}
inline double ACOSteiner::getMutationProbability() const {return this->mutation_probability;}
inline double ACOSteiner::getEvaporationCofficient() const {return this->evaporation_cofficient;}
inline double ACOSteiner::getBasicMoveRatio() const {return this->basic_move_ratio;}
inline ll ACOSteiner::getTableCapacity() const {return this->table->getCapacity();}

inline const ACOTable& ACOSteiner::getACOTable() const {return *this->table;}
inline const QuadTree& ACOSteiner::getQuadTree(int index) const {return this->qtworld[index];}

//以下でprivate関数

void ACOSteiner::countTime(){
  for(const Ant *a:*(this->table))a->evaporate(this->evaporation_cofficient);
}
