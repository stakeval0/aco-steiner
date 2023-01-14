#ifndef _QUAD_TREE_HPP_
#define _QUAD_TREE_HPP_
#include<vector>
#include<array>
#include<map>
#include<set>
#include<functional>
using namespace std;

//NOTE: 本当はTは参照型で受け渡しするべきだが、Ant*とintしか許可する予定がないので値渡しする

template<class T>
struct QuadTreeNode{
  const array<double,2> &point;
  int index;
  T value;
  QuadTreeNode(const array<double,2> &point,int index,T value)
    : point(point),index(index),value(value){}
};

template<class T>
class QuadTree {
  private:
    array<double,2> offset_v,size_v;
    int level;
    double unit_width;
    double unit_height;
    map<uint,set<QuadTreeNode<T>>>quad_tree;
    uint separate(ushort n) const;
    bool inWorld(double x,double y) const;
    uint mortonNumber(double x, double y) const;
    void searchMorton(uint morton,int search_depth,
                      const function<bool(const array<double,2>&)> &filter,
                      vector<QuadTreeNode<T>> &buf) const;
  public:
    QuadTree(double width, double height);
    QuadTree(double width, double height, int level);
    QuadTree(const array<double,2> &offset,const array<double,2> &size);
    QuadTree(const array<double,2> &offset,const array<double,2> &size,int level);
    void addRoute(const vector<array<double,2>> &route,T a);
    void removeRoute(const vector<array<double,2>> &route,T a);
    vector<QuadTreeNode<T>> reachablePoints(
        double cx,double cy,double width,double height) const;
    vector<QuadTreeNode<T>> reachablePoints(
        double cx,double cy,double width,double height,
        const function<bool(const array<double,2>&)> &filter) const;
    double width() const;
    double height() const;
    const array<double,2>& size() const;
    const array<double,2>& minPoint() const;
};

#endif//_QUAD_TREE_HPP_