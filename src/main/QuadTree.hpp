#include<vector>
#include<array>
#include<map>
#include<bitset>
#include"Ant.hpp"
using namespace std;
using ull=unsigned long long;

// 4分木のモデル
class QuadTree {
  private:
    uint width;
    uint height;
    int level;
    double unit_width;
    double unit_height;
    map<ll,set<pair<const array<double,2>&,const Ant*>>>quad_tree;
    ull separate(uint n);
    ull getMortonNumber(uint x, uint y);
    void searchSeparatedMorton(uint left_up_x,uint left_up_y,uint right_bottom_x,uint right_bottom_y,vector<pair<const array<double,2>&,const Ant*>> &buf);
    //void addPoint(const array<double,2> &object,const Ant *a);// array<double,2>を追加する
  public:
    QuadTree(double width, double height, int level);
    void addRoute(const vector<array<double,2>> &route,const Ant *a);
    vector<pair<const array<double,2>&,const Ant*>>* reachablePoints(double cx,double cy,double width,double height);
};
