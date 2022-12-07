#ifndef _ACO_TABLE_HPP_
#define _ACO_TABLE_HPP_
#include<array>
#include<queue>
#include<functional>
#include"Ant.hpp"
using namespace std;
using ll=long long;

class ACOTable : public priority_queue<Ant,vector<Ant>,
                                       function<bool(Ant&,
                                       Ant&)>> {
  private:
    ll time=0,maxSize=16;
  public:
    ACOTable();
    Ant selectAnt();
    void setTime(ll time);
    void setTableSize(ll size);
    double costVariance();
    void push(const Ant &a);
};


#endif//_ACO_TABLE_HPP_