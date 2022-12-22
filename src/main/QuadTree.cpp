#include"QuadTree.hpp"

// 4分木のモデル
QuadTree::QuadTree(double width, double height, int level) {
  this->width=ceil(width);
  this->height=ceil(height);
  this->level=level;
  this->unit_width=this->width/(double)(1<<this->level);
  this->unit_height=this->height/(double)(1<<this->level);
}

void QuadTree::addRoute(const vector<array<double,2>> &route,const Ant *a){
  for(int i=0;i<route.size();i++){
    const array<double,2> &point=route[i];
    if(point[0]<=0||point[1]<=0||point[0]>=this->width||point[1]>=this->height){
      fprintf(stderr,"QuadTree: Irregular point (%g, %g) was observed.\n",point[0],point[1]);
      continue;
    }
    uint x=(uint)(point[0]/this->unit_width);
    uint y=(uint)(point[1]/this->unit_height);
    ll morton_num=getMortonNumber(x,y);
    quad_tree[morton_num].insert({point,a});
  }
}

vector<pair<const array<double,2>&,const Ant*>>* QuadTree::reachablePoints(double cx,double cy,double width,double height){
  vector<pair<const array<double,2>&,const Ant*>>* ret=new vector<pair<const array<double,2>&,const Ant*>>;
}

//以下でprivate関数

ull QuadTree::separate(uint n) {
  ull lln=n,ret=0;
  for(int i=0;i<sizeof(n)*8;i++){
    ret+=(lln&(uint)1<<i)<<i;
  }
  return ret;
}

ull QuadTree::getMortonNumber(uint x, uint y) {
  return (separate(x) | separate(y) << 1);
}

void searchSeparatedMorton(uint left_up_x,uint left_up_y,uint right_bottom_x,uint right_bottom_y,vector<pair<const array<double,2>&,const Ant*>> &buf){

}

int getShiftNum(int morton1, int morton2) {
    int xclusive_or = morton1 ^ morton2;
    int num = 0;
    for (int i = 0; i < 32; i+=2) {
      if (((xclusive_or >> i) & 0x00000003) != 0) {
        num = i + 2;
      }
    }
    return num;
  }