#include"Ant.hpp"
#include"ACOSteiner.hpp"
#include"ACOTable.hpp"
#include"QuadTree.hpp"

const double MIN_SPACE_RATIO_DEFAULT=1.0/100,MAX_SPACE_RATIO_DEFAULT=1.0/10;

ACOSteiner::ACOSteiner(const vector<array<double,2>> &points){
  this->table=new ACOTable();
  double xmin=0,xmax=0,ymin=0,ymax=0;
  for(int i=0;i<points.size();i++){
    xmin=min(xmin,points[i][0]);xmax=max(xmax,points[i][0]);
    ymin=min(ymin,points[i][1]);ymax=max(ymax,points[i][1]);
  }
  this->points_offset={xmin,ymin};
  for(int i=0;i<points.size();i++){
    this->points[i][0]=points[i][0]-xmin;
    this->points[i][1]=points[i][1]-ymin;
  }
  double width=xmax-xmin,height=ymax-ymin;
  int num_of_qt=points.size()-1;
  this->qtworld.reserve(num_of_qt);
  for(int i=0;i<num_of_qt;i++){
    this->qtworld.emplace_back(width,height);
  }
  double diagnoal=sqrt(width*width+height*height);
  this->min_space=diagnoal*MIN_SPACE_RATIO_DEFAULT;
  this->max_space=diagnoal*MAX_SPACE_RATIO_DEFAULT;
}

void ACOSteiner::search(){
  Ant *new_ant=new Ant(*this,false);
  this->table->setTime(this->time);
  this->table->insert(new_ant);
  //4分木の世界へ登録
  for(int i=0;i<new_ant->numOfPoints();i++){
    this->qtworld[i].addRoute(new_ant->getRoute(i)->second,new_ant);
  }
  Ant *dropout_ant=this->table->dropout();
  if(dropout_ant){
    for(int i=0;i<dropout_ant->numOfPoints();i++){
      this->qtworld[i].removeRoute(dropout_ant->getRoute(i)->second,dropout_ant);
    }
    delete dropout_ant;
  }
  this->time++;//最後に書く
}

ll ACOSteiner::getTime() const {return this->time;}
void ACOSteiner::setMinSpace(double d){this->min_space=d;}
void ACOSteiner::setMaxSpace(double d){this->max_space=d;}
double ACOSteiner::getMinSpace() const {return this->min_space;}
double ACOSteiner::getMaxSpace() const {return this->max_space;}
void ACOSteiner::setTableSize(ll size){this->table->setTableSize(size);}
const ACOTable* ACOSteiner::getACOTable() const {return this->table;}
