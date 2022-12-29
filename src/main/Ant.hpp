#ifndef _ANT_HPP_
#define _ANT_HPP_
#include<vector>
#include<array>
#include<cmath>
#include<memory>
using namespace std;
using ll=long long;

class ACOSteiner;

class Ant{
  protected:
    ll birth_time;
    double all_cost;
    vector<pair<int,shared_ptr<vector<array<double,2>>>>>path;//pair<int,vec>のintで戻るノード数を管理する
    void constructOneRoute(const ACOSteiner &w,const Ant *base_ant,int index);
    void searchNewRoute(const ACOSteiner &w,ll current_time);
  public:
    Ant(const ACOSteiner &world,bool init);
    double pheromone(ll current_time) const;
    double getAllCost() const;
    int getBackTimesOnRoute(int index) const;
    const vector<array<double,2>>& getRelayPointsOnRoute(int index) const;
    const tuple<int,const vector<array<double,2>> &> getRoute(int index) const;//互換性のために残すが非推奨
    int numOfPoints() const;
    //static Ant* searchNewRoute(ACOSteiner world);
};

#endif//_ANT_HPP_