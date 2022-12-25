#include<iostream>
#include<array>
#include<functional>
#include"ACOSteiner.hpp"
#include"ACOTable.hpp"
#include"Ant.hpp"

ACOSteiner::ACOSteiner(double min_space,double max_space){
  this->min_space=min_space;
  this->max_space=max_space;
  this->table=new ACOTable();
}
void ACOSteiner::search(){
  Ant *a=new Ant(*this,false);
  this->table->setTime(this->time);
  this->table->insert(a);
  //4分木の世界へ登録
  this->time++;//最後に書く
}
ll ACOSteiner::getTime() const {return this->time;}
void ACOSteiner::setMinSpace(double d){this->min_space=d;}
void ACOSteiner::setMaxSpace(double d){this->max_space=d;}
double ACOSteiner::getMinSpace() const {return this->min_space;}
double ACOSteiner::getMaxSpace() const {return this->max_space;}
void ACOSteiner::setTableSize(ll size){this->table->setTableSize(size);}
const ACOTable* ACOSteiner::getACOTable() const {return this->table;}
