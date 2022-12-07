#ifndef _ACO_TABLE_HPP_
#define _ACO_TABLE_HPP_
#include<array>
#include<set>
#include<functional>
#include"Ant.hpp"
using namespace std;
using ll=long long;

//最良と最悪どちらも取り出したいのでmultiset方式
class ACOTable : public multiset<Ant,function<bool(Ant&,Ant&)>> {
  private:
    ll time=0,maxSize=16;
  public:
    ACOTable();
    Ant selectAnt();
    void setTime(ll time);
    void setTableSize(ll size);
    double costVariance();
    void insert(const Ant &a);
};


#endif//_ACO_TABLE_HPP_