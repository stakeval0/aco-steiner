#include<iostream>
#include<array>
#include<functional>
#include"ACOSteiner.hpp"

ACOSteiner::ACOSteiner(double minSpace,double maxSpace){
  this->minSpace=minSpace;
  this->maxSpace=maxSpace;
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
void ACOSteiner::setMinSpace(double d){this->minSpace=d;}
void ACOSteiner::setMaxSpace(double d){this->maxSpace=d;}
double ACOSteiner::getMinSpace() const {return this->minSpace;}
double ACOSteiner::getMaxSpace() const {return this->maxSpace;}
void ACOSteiner::setTableSize(ll size){this->table->setTableSize(size);}
const ACOTable* ACOSteiner::getACOTable() const {return this->table;}
