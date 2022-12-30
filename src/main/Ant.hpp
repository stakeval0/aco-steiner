#ifndef _ANT_HPP_
#define _ANT_HPP_
#include<vector>
#include<array>
#include<memory>
using namespace std;
using ll=long long;

class ACOSteiner;

class Ant{
  protected:
    const ll BIRTH_TIME;
    double all_cost;//constは付けられないが、生成以降、少なくともpublic関数では定数として扱って良い
    mutable double pheromone_v;
    vector<pair<int,shared_ptr<vector<array<double,2>>>>>path;//pair<int,vec>のintで戻るノード数を管理する
    void addRandVecToOneRoute(const ACOSteiner &w,const Ant *base_ant,int index);
    void searchNewRoute(const ACOSteiner &w,ll current_time);
    void constructFirstAnt(const ACOSteiner &w);
  public:
    Ant(const ACOSteiner &world);
    void evaporate(double evaporation_cofficient) const;
    double pheromone() const;
    double allCost() const;
    int getBackTimesOnRoute(int index) const;
    const vector<array<double,2>>& getRelayPointsOnRoute(int index) const;
    const tuple<int,const vector<array<double,2>> &> getRoute(int index) const;//互換性のために残すが非推奨
    int numOfPoints() const;
    //static Ant* searchNewRoute(ACOSteiner world);
};

#endif//_ANT_HPP_