#ifndef _ANT_HPP_
#define _ANT_HPP_
#include<vector>
#include<array>
#include<memory>
using namespace std;
using ll=long long;
using v2d=array<double,2>;//TODO: 後でクラスを作り直してリファクタリングしたい

class ACOSteiner;

struct SingleRoute;

class Ant{
  protected:
    const ll BIRTH_TIME;
    const double MIN_DISTANCE,MAX_DISTANCE;
    double total_cost,total_length;//constは付けられないが、生成以降、少なくともpublic関数では定数として扱って良い
    mutable double pheromone_v;
    vector<shared_ptr<SingleRoute>>path;//pair<int,vec>のintで戻るノード数を管理する
    void properPushBack(vector<v2d> &v,const v2d &e,const v2d &min_point,const v2d &max_point);
    void addRandVecToOneRoute(const ACOSteiner &w,const Ant *base_ant,int index);
    void constructModifiedRoute(const ACOSteiner &w,ll current_time);
    void constructFirstRoute(const ACOSteiner &w);
  public:
    Ant(const ACOSteiner &world);
    void evaporate(double evaporation_cofficient) const;
    double pheromone() const;
    double cost() const;
    double length() const;
    const vector<v2d>& getRelayPointsOnRoute(int index) const;
    int numOfPoints() const;
};

#endif//_ANT_HPP_