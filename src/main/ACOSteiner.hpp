#ifndef _ACO_STEINER_HPP_
#define _ACO_STEINER_HPP_
#include<array>
#include<queue>
#include<functional>
#include"Ant.hpp"
#include"ACOTable.hpp"
using namespace std;
using ll =long long;

class Ant;
class ACOTable;

class ACOSteiner{
  protected:
    ll time=0;
    double minSpace,maxSpace;
    ACOTable* table;
  public:
    ACOSteiner(double minSpace,double maxSpace);
    void search();
    ll getTime() const;
    void setMinSpace(double d);
    void setMaxSpace(double d);
    double getMinSpace() const;
    double getMaxSpace() const;
    void setTableSize(ll size);
    const ACOTable* getACOTable() const;
};


#endif//_ACO_STEINER_HPP_