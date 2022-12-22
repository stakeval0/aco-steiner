#ifndef _ACO_TABLE_HPP_
#define _ACO_TABLE_HPP_
#include<array>
#include<set>
#include<functional>
#include"Ant.hpp"
using namespace std;
using ll=long long;

class Ant;

//最良と最悪どちらも取り出したいのでmultiset方式
class ACOTable : public multiset<Ant*,function<bool(Ant*,Ant*)>> {
  private:
    ll time=0,max_size=16;
  public:
    ACOTable();
    Ant selectAnt();
    void setTime(ll time);
    void setTableSize(ll size);
    double costVariance();
    void insert(Ant *a);
    void updatePheromone();
    //double pheromoneSum() const;//出来ればupdatePheromone()時に計算して変数として保存しておきたい
};

#endif//_ACO_TABLE_HPP_