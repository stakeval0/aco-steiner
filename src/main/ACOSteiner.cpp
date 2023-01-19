#include<iostream>
#include<cmath>
#include<type_traits>
#include"Util.hpp"
#include"Ant.hpp"
#include"ACOSteiner.hpp"
#include"QuadTree.hpp"
#include"ACOTable.hpp"

static constexpr double MIN_SPACE_RATIO_DEFAULT=1.0/100,MAX_SPACE_RATIO_DEFAULT=1.0/10;

static void printArgumentErrorMessage(const char *func_name,const char *correct_argument_condition){
  fprintf(stderr,"%s: argument needs to be %s.\n",func_name,correct_argument_condition);
}

ACOSteiner::ACOSteiner(const vector<array<double,2>> &points){
  this->table=new ACOTable();
  this->cost_function=static_cast<double(*)(
      const array<double,2>&,const array<double,2>&)>(euclid);
  double xmin=0,xmax=0,ymin=0,ymax=0;
  this->points.resize(points.size());
  for(int i=0;i<points.size();i++){
    this->points[i][0]=points[i][0];
    this->points[i][1]=points[i][1];
    xmin=min(xmin,points[i][0]);xmax=max(xmax,points[i][0]);
    ymin=min(ymin,points[i][1]);ymax=max(ymax,points[i][1]);
  }
  this->pheromone_cofficient=0;
  int num_of_route=points.size()-1;
  array<double,2> points_offset{xmin,ymin},size{xmax-xmin,ymax-ymin};
  this->qtworld.reserve(num_of_route);
  for(int i=0;i<num_of_route;i++){
    this->qtworld.emplace_back(points_offset,size);
    this->pheromone_cofficient+=cost_function(points[i],points[i+1]);
  }
  double diagnoal=euclid(size);
  this->min_distance=diagnoal*MIN_SPACE_RATIO_DEFAULT;
  this->max_distance=diagnoal*MAX_SPACE_RATIO_DEFAULT;
}

ACOSteiner::~ACOSteiner(){
  for(auto &e:*this->table){
    delete e;
  }
  delete this->table;//ACOTableをunique_ptrにすれば書かなくて良くなる
}

void ACOSteiner::search(){
  countTime();
  Ant *new_ant=new Ant(*this);
  this->table->insert(new_ant);
  //cout<<new_ant->pheromone()<<endl;
  //4分木の世界へ登録
  for(int i=0;i<new_ant->routeNum();i++){
    this->qtworld[i].addRoute(new_ant->getRoute(i),new_ant);
  }
  //最悪解の削除
  const Ant *dropout_ant=this->table->dropout();
  if(dropout_ant){
    for(int i=0;i<dropout_ant->routeNum();i++){
      this->qtworld[i].removeRoute(dropout_ant->getRoute(i),dropout_ant);
    }
    //設計段階でここでdeleteすれば必要十分である事が証明されているが、後でスマートポインタにしても良いかも知れない
    delete dropout_ant;
  }
}

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

void ACOSteiner::setTableCapacity(ll size){this->table->setCapacity(size);}
ll ACOSteiner::getTableCapacity() const { return this->table->getCapacity(); }
const QuadTreeAnt& ACOSteiner::getQuadTreeAnt(int index) const {
  return this->qtworld[index];
};

//以下でprivate関数

void ACOSteiner::countTime(){
  for(const Ant *a:*(this->table))a->evaporate(this->evaporation_cofficient);
}
