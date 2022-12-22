#include<random>
#include"Ant.hpp"
using namespace std;

void Ant::searchNewRoute(const ACOSteiner &world,ll current_time){
  const ACOTable *TABLE=world.getACOTable();
  double pheromone_sum=0;
  for(const auto &e:*TABLE)pheromone_sum+=e->pheromone(current_time);
  random_device seedGen;
  default_random_engine engine(seedGen());
  uniform_real_distribution<> dist(0,pheromone_sum);
  double rand=dist(engine),sum=0;
  Ant *baseAnt;
  for(const auto &e:*TABLE){
    sum+=e->pheromone(this->birth_time);
    if(sum>=rand){baseAnt=e;break;}
  }
  //TODO: 実装!
}

Ant::Ant(const ACOSteiner &world,bool init){
  this->birth_time=world.getTime();
  if(!init)searchNewRoute(world,this->birth_time);
}

/* Ant* Ant::searchNewRoute(ACOSteiner world){
   Ant *ret=new Ant();
 } */