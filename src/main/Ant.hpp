#ifndef _ANT_HPP_
#define _ANT_HPP_
#include<vector>
#include<array>
#include<cmath>
using namespace std;
using ll=long long;

class ACOSteiner;

class Ant{
  protected:
    ll birth_time;
    double all_cost;
    vector<pair<int,vector<array<double,2>>>>path;//pair<int,vec>のintで戻るノード数を管理する
    static double (*object_function)(const array<double,2> &,const array<double,2> &);//目的関数
    void searchNewRoute(const ACOSteiner &w,ll current_time);
  public:
    Ant(const ACOSteiner &world,bool init);
    double pheromone(ll current_time);
    double getAllCost();
    //static Ant* searchNewRoute(ACOSteiner world);
};

static double euclid(const array<double,2> &a,const array<double,2> &b){
  double dx=b[0]-a[0],dy=b[1]-a[1];
  return sqrt(dx*dx+dy*dy);
}
double (*Ant::object_function)(const array<double,2> &,const array<double,2> &)=&euclid;

#endif//_ANT_HPP_