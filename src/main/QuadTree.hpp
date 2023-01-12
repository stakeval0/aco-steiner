#ifndef _QUAD_TREE_HPP_
#define _QUAD_TREE_HPP_
#include<vector>
#include<array>
#include<map>
#include<set>
using namespace std;

//NOTE: 本当はTは参照型で受け渡しするべきだが、Ant*とintしか許可する予定がないので値渡しする

template<class T>
class QuadTree {
  private:
    double width;
    double height;
    int level;
    double unit_width;
    double unit_height;
    map<uint,set<pair<const array<double,2>*,T>>>quad_tree;
    uint separate(ushort n);
    bool inWorld(double x,double y);
    uint mortonNumber(double x, double y);
    void searchMorton(uint morton,int search_depth,vector<pair<const array<double,2>&,T>> &buf);
    //void addPoint(const array<double,2> &object,T a);// array<double,2>を追加する
  public:
    QuadTree(double width, double height);
    QuadTree(double width, double height, int level);
    void addRoute(const vector<array<double,2>> &route,T a);
    void removeRoute(const vector<array<double,2>> &route,T a);
    vector<pair<const array<double,2> &,T>>* reachablePoints(double cx,double cy,double width,double height);
};

#endif//_QUAD_TREE_HPP_