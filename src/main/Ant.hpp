#ifndef _ANT_HPP_
#define _ANT_HPP_
#include<vector>
#include<array>
#include<memory>
using namespace std;
using ll=long long;
using v2d=array<double,2>;//TODO: 後でクラスを作り直してリファクタリングしたい

class ACOSteiner;

class Ant{
  protected:
    const ll BIRTH_TIME;
    const double MIN_DISTANCE,MAX_DISTANCE;
    double all_cost;//constは付けられないが、生成以降、少なくともpublic関数では定数として扱って良い
    mutable double pheromone_v;
    vector<pair<tuple<int,int,double>,shared_ptr<vector<v2d>>>>path;//pair<int,vec>のintで戻るノード数を管理する
    void properPushBack(vector<v2d> &v,const v2d &e);
    void addRandVecToOneRoute(const ACOSteiner &w,const Ant *base_ant,int index);
    void searchNewRoute(const ACOSteiner &w,ll current_time);
    void constructFirstAnt(const ACOSteiner &w);
  public:
    Ant(const ACOSteiner &world);
    void evaporate(double evaporation_cofficient) const;
    double pheromone() const;
    double allCost() const;
    //tuple<int,int,double> getBackTimesOnRoute(int index) const;
    const vector<v2d>& getRelayPointsOnRoute(int index) const;
    const pair<tuple<int,int,double>,const vector<v2d> &> getRoute(int index) const;//互換性のために残すが非推奨
    int numOfPoints() const;
    //static Ant* searchNewRoute(ACOSteiner world);
};

#endif//_ANT_HPP_