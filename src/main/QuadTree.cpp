#include"QuadTree.hpp"

// 4分木のモデル
QuadTree::QuadTree(double width, double height, int level) {
  this->width=ceil(width);
  this->height=ceil(height);
  this->level=level;
  this->unitWidth=this->width/(double)(1<<this->level);
  this->unitHeight=this->height/(double)(1<<this->level);
}

void QuadTree::addRoute(const vector<array<double,2>> &route,const Ant *a){
  for(int i=0;i<route.size();i++){
    const array<double,2> &point=route[i];
    if(point[0]<=0||point[1]<=0||point[0]>=this->width||point[1]>=this->height){
      fprintf(stderr,"QuadTree: Irregular point (%g, %g) was observed.\n",point[0],point[1]);
      continue;
    }
    uint x=(uint)(point[0]/this->unitWidth);
    uint y=(uint)(point[1]/this->unitHeight);
    ll mortonNum=getMortonNumber(x,y);
    quadTree[mortonNum].insert({point,a});
  }
}

vector<pair<const array<double,2>&,const Ant*>>* QuadTree::reachablePoints(double cx,double cy,double width,double height){

}

//以下でprivate関数

ull QuadTree::separate(uint n) {
  ull lln=n,ret=0;
  for(int i=0;i<sizeof(uint)*8;i++){
    ret+=(lln&(uint)1<<i)<<i;
  }
  return ret;
}

ull QuadTree::getMortonNumber(uint x, uint y) {
  return (separate(x) | separate(y) << 1);
}