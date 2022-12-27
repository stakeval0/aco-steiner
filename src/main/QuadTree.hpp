#ifndef _QUAD_TREE_HPP_
#define _QUAD_TREE_HPP_
#include<vector>
#include<array>
#include<map>
#include<bitset>
#include<set>
#include<cmath>
//#include"Ant.hpp"
using namespace std;

class Ant;

class QuadTree {
  private:
    double width;
    double height;
    int level;
    double unit_width;
    double unit_height;
    map<uint,set<pair<const array<double,2>*,const Ant*>>>quad_tree;
    uint separate(ushort n);
    bool inWorld(double x,double y);
    uint mortonNumber(double x, double y);
    void searchMorton(uint morton,int search_depth,vector<pair<const array<double,2>&,const Ant*>> &buf);
    //void addPoint(const array<double,2> &object,const Ant *a);// array<double,2>を追加する
  public:
    static const uint MASK_X=0x55555555,MASK_Y=MASK_X<<1;
    QuadTree(double width, double height);
    QuadTree(double width, double height, int level);
    void addRoute(const vector<array<double,2>> &route,const Ant *a);
    void removeRoute(const vector<array<double,2>> &route,const Ant *a);
    vector<pair<const array<double,2> &,const Ant*>>* reachablePoints(double cx,double cy,double width,double height);
};

#endif//_QUAD_TREE_HPP_