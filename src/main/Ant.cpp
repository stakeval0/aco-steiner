#include<random>
#include"Ant.hpp"
using namespace std;

void Ant::searchNewRoute(const ACOSteiner &world,ll currentTime){
  const ACOTable *TABLE=world.getACOTable();
  double pheromoneSum=0;
  for(const auto &e:*TABLE)pheromoneSum+=e->pheromone(currentTime);
  random_device seedGen;
  default_random_engine engine(seedGen());
  uniform_real_distribution<> dist(0,pheromoneSum);
  double rand=dist(engine),sum=0;
  Ant *baseAnt;
  for(const auto &e:*TABLE){
    sum+=e->pheromone(this->birthTime);
    if(sum>=rand){baseAnt=e;break;}
  }
  //TODO: 実装!
}

Ant::Ant(const ACOSteiner &world,bool init){
  this->birthTime=world.getTime();
  if(!init)searchNewRoute(world,this->birthTime);
}

/* Ant* Ant::searchNewRoute(ACOSteiner world){
   Ant *ret=new Ant();
 } */