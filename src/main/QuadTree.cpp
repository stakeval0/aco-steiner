#include<iostream>
#include<stack>
#include<cmath>
#include"QuadTree.hpp"

static constexpr uint MASK_X=0x55555555,MASK_Y=MASK_X<<1;

//NOTE: 深さの最小値は、最上位2bitが一致しない時に0
static int mortonDepth(uint morton1, uint morton2) {
  uint xclusive_or = morton1 ^ morton2;
  int i;
  for(i=sizeof(morton1)*8-2;i>=0&&!((xclusive_or>>i)&0x3);i-=2);
  return (sizeof(morton1)*8-i-2)/2;
}

static uint mortonInDepth(uint morton,uint depth){
  return morton&~(__UINT32_MAX__>>depth*2);
}

static uint mortonJustInDepth(uint morton,uint depth){
  return morton&~(0x3<<(sizeof(uint)*8-2*depth));
}

static uint moveToCenterInParentMorton(uint morton,uint fixed_depth,uint mask_xy){
  uint depth_bit=(morton>>(sizeof(uint)*8-fixed_depth*2))&0x3;
  bool x=!(depth_bit&1),y=!(depth_bit&2);
  uint direct_mask=x*MASK_X^y*MASK_Y;//それぞれの軸方向どちらに寄るか
  return (morton&~(mask_xy>>fixed_depth*2))^(direct_mask&mask_xy>>2*fixed_depth);
}

static void pushNewMortonArea(stack<array<uint,3>> &s,uint left_up_morton,uint right_bottom_morton){
  array<uint,3>tmp;
  tmp[0]=left_up_morton;tmp[1]=right_bottom_morton;
  tmp[2]=mortonDepth(tmp[0],tmp[1]);
  s.push(tmp);
}

template<class T>
bool operator<(const QuadTreeNode<T> &n1,const QuadTreeNode<T> &n2){
  if(&(n1.point)!=&(n2.point))return &(n1.point)<&(n2.point);
  if(n1.index!=n2.index)return n1.index<n2.index;
  return n1.value<n2.value;
}

/*
  NOTE: QuadTreeNodeの等価演算子を、座標の等価をポインタとして判断する案があったが、
        面倒くさいし特に支障もないので差し当たって定義しない。
*/

// 4分木のモデル
template<class T>
QuadTree<T>::QuadTree(double width, double height, int level) {
  new (this) QuadTree({0,0},{width,height},level);
}

template<class T>
QuadTree<T>::QuadTree(double width,double height){
  new (this) QuadTree(width,height,sizeof(uint)*8/2);
}

template<class T>
QuadTree<T>::QuadTree(const array<double,2> &offset,const array<double,2> &size){
  new (this) QuadTree(offset,size,sizeof(uint)*8/2);
}

template<class T>
QuadTree<T>::QuadTree(const array<double,2> &offset,const array<double,2> &size,int level) {
  if(level>sizeof(ushort)*8){fprintf(stderr,"%s: level is too big.\n",__PRETTY_FUNCTION__);exit(EXIT_FAILURE);}
  this->offset_v=offset;
  this->size_v=size;
  this->level=level;
  this->unit_width=this->size_v[0]/(double)(1<<this->level);
  this->unit_height=this->size_v[1]/(double)(1<<this->level);
}

template<class T>
void QuadTree<T>::addRoute(const vector<array<double,2>> &route,T a){
  for(int i=0;i<route.size();i++){
    addPoint(route[i],i,a);
  }
}

template<class T>
void QuadTree<T>::removeRoute(const vector<array<double,2>> &route,T a){
  for(int i=0;i<route.size();i++){
    removePoint(route[i],i,a);
  }
}

template <class T>
void QuadTree<T>::addPoint(const array<double, 2> &point, int index, T a) {
  if(!inWorld(point[0],point[1]))return;
  int morton_num=mortonNumber(point[0],point[1]);
  quad_tree[morton_num].emplace(point,index,a);
}

template <class T>
void QuadTree<T>::removePoint(const array<double, 2> &point, int index, T a) {
  if(!inWorld(point[0],point[1]))return;
  int morton_num=mortonNumber(point[0],point[1]);
  quad_tree[morton_num].erase({point,index,a});
}

template<class T>
vector<QuadTreeNode<T>> QuadTree<T>::reachablePoints(
    double cx,double cy,double width,double height) const {
  const auto filter=[cx,cy,width,height](const array<double,2>& p){
    return abs(p[0]-cx)<=width/2&&abs(p[1]-cy)<=height/2;
  };
  return reachablePoints(cx,cy,width,height,filter);
}

template<class T>
vector<QuadTreeNode<T>> QuadTree<T>::reachablePoints(
    double cx,double cy,double width,double height,
    const function<bool(const array<double,2>&)> &filter) const {
  vector<QuadTreeNode<T>> ret;
  stack<array<uint,3>>s;//left_up_morton,right_down_morton,depth
  double hw=width/2,hh=height/2;
  double p1_x=max(cx-hw,0.),p1_y=max(cy-hh,0.),
         p2_x=min(cx+hw,this->width()),p2_y=min(cy+hh,this->height());
  const int MAX_DEPTH=ceil(log2(this->width()*this->height()/(width*height))/2);
  array<uint,3>tmp;
  tmp[0]=mortonNumber(p1_x,p1_y);tmp[1]=mortonNumber(p2_x,p2_y);
  tmp[2]=mortonDepth(tmp[0],tmp[1]);
  s.push(tmp);
  while(!s.empty()){
    tmp=s.top();s.pop();
    if(tmp[2]>=MAX_DEPTH){
      searchMorton(tmp[0],tmp[2],filter,ret);
      //cout<<tmp[0]<<' '<<tmp[1]<<endl;
      continue;
    }
    uint next_depth=tmp[2]+1;
    uint p1_masked_x=tmp[0]&MASK_X,p1_masked_y=tmp[0]&MASK_Y;
    uint p2_masked_x=tmp[1]&MASK_X,p2_masked_y=tmp[1]&MASK_Y;
    array<uint,4>current_mortones={
      tmp[0],p2_masked_x^p1_masked_y,p1_masked_x^p2_masked_y,tmp[1]
    };
    if(mortonInDepth(current_mortones[0],next_depth)==mortonInDepth(current_mortones[1],next_depth)){
      //この時探索領域は縦長のように思える
      pushNewMortonArea(s,
        moveToCenterInParentMorton(current_mortones[2],next_depth,MASK_Y),
        current_mortones[3]
      );
      pushNewMortonArea(s,
        current_mortones[0],
        moveToCenterInParentMorton(current_mortones[1],next_depth,MASK_Y)
      );
    }else if(mortonInDepth(current_mortones[0],next_depth)==mortonInDepth(current_mortones[2],next_depth)){
      //この時探索領域は横長のように思える
      pushNewMortonArea(s,
        moveToCenterInParentMorton(current_mortones[1],next_depth,MASK_X),
        current_mortones[3]
      );
      pushNewMortonArea(s,
        current_mortones[0],
        moveToCenterInParentMorton(current_mortones[2],next_depth,MASK_X)
      );
    }else{
      //この時current_mortonesの要素は全て異なる子領域に存在する
      pushNewMortonArea(s,
        moveToCenterInParentMorton(current_mortones[3],next_depth,__UINT32_MAX__),
        current_mortones[3]
      );
      pushNewMortonArea(s,
        moveToCenterInParentMorton(current_mortones[2],next_depth,MASK_Y),
        moveToCenterInParentMorton(current_mortones[2],next_depth,MASK_X)
      );
      pushNewMortonArea(s,
        moveToCenterInParentMorton(current_mortones[1],next_depth,MASK_X),
        moveToCenterInParentMorton(current_mortones[1],next_depth,MASK_Y)
      );
      pushNewMortonArea(s,
        current_mortones[0],
        moveToCenterInParentMorton(current_mortones[0],next_depth,__UINT32_MAX__)
      );
    }
  }
  return ret;
}

template<class T>
inline double QuadTree<T>::width() const {return this->size_v[0];}

template<class T>
inline double QuadTree<T>::height() const {return this->size_v[1];}

template <class T>
inline const array<double,2>& QuadTree<T>::size() const {return this->size_v;}

template <class T>
inline const array<double,2>& QuadTree<T>::minPoint() const {return this->offset_v;}

//以下でprivate関数

template<class T>
uint QuadTree<T>::separate(ushort n) const {
  uint lln=n,ret=0;
  for(int i=0;i<sizeof(n)*8;i++){
    ret+=(lln&((uint)1<<i))<<i;
  }
  return ret;
}

template<class T>
bool QuadTree<T>::inWorld(double x,double y) const {
  double x_in_range=x-this->offset_v[0],y_in_range=y-this->offset_v[1];
  if(x_in_range<0||y_in_range<0||x_in_range>this->width()||y_in_range>this->height()){
    fprintf(stderr,"%s: Irregular point (%g, %g) was observed.\n",__PRETTY_FUNCTION__,x,y);
    return false;
  }
  return true;
}

template<class T>
uint QuadTree<T>::mortonNumber(double x, double y)  const {
  //x==this->widthになるとオーバーフローするのでその対策
  ushort i=min((int)((x-this->offset_v[0])/this->unit_width),(1<<this->level)-1);
  ushort j=min((int)((y-this->offset_v[1])/this->unit_height),(1<<this->level)-1);
  uint ret=(separate(i) | separate(j) << 1);
  return ret;
}

template<class T>
void QuadTree<T>::searchMorton(uint morton,int search_depth,const function<bool(const array<double,2>&)> &filter,vector<QuadTreeNode<T>> &buf) const {
  const uint LOWER_BIT_MAX=__UINT32_MAX__>>(2*search_depth),//ここをbitsetに付いてきたUINT32_MAXにして、3項演算子でない状態でやっていたら何故か値がsearch_depth==16でUINT32_MAXになった
             MORTON_HIGHER_BIT=morton&~LOWER_BIT_MAX,MORTON_MAX=MORTON_HIGHER_BIT+LOWER_BIT_MAX;
  auto e=this->quad_tree.lower_bound(MORTON_HIGHER_BIT);
  //cout<<MORTON_HIGHER_BIT<<":\n";
  if(e!=this->quad_tree.end()&&e->first<MORTON_HIGHER_BIT)e++;
  for(;e!=this->quad_tree.end()&&e->first<=MORTON_MAX;e++){
    //cout<<e->first<<endl;
    for(const auto &g:e->second){//NOTE: 本当はsetをbufに詰められる様にした方が速そうだが、簡単なのでこうする
      if(filter(g.point))buf.emplace_back(g);
    }
  }
}

template class QuadTree<const int>;

class Ant;
template class QuadTree<const Ant*>;