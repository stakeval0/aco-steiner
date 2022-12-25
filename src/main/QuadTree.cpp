#include<stack>
#include"QuadTree.hpp"

//NOTE: 深さの最小値は、最上位2bitが一致しない時に0
static int mortonDepth(uint morton1, uint morton2) {
  uint xclusive_or = morton1 ^ morton2;
  int i;
  for(i=sizeof(morton1)-2;i>=0&&((xclusive_or>>i)&0x3);i-=2);
  return (sizeof(morton1)-i+2)/2;
}

static uint mortonInDepth(uint morton,uint depth){
  return morton&~(UINT32_MAX>>depth*2);
}

static uint mortonJustInDepth(uint morton,uint depth){
  return morton&~(0x3<<(sizeof(uint)*8-2*depth));
}

static uint moveToParentMortonCenter(uint morton,uint move_depth,uint mask_xy){
  uint depth_bit=(morton>>(sizeof(uint)-move_depth*2))&0x3;
  bool x=!(depth_bit&1),y=!(depth_bit&2);
  uint direct_mask=x*QuadTree::MASK_X^y*QuadTree::MASK_Y;//それぞれの軸方向どちらに寄るか
  return (morton&~(mask_xy>>move_depth*2))^(direct_mask&mask_xy>>2*move_depth);
}

//TODO: 後でlevelを消す
// 4分木のモデル
QuadTree::QuadTree(double width, double height, int level) {
  if(level>sizeof(ushort)){fprintf(stderr,"QuadTree: level is too big.\n");exit(EXIT_FAILURE);}
  this->width=ceil(width);
  this->height=ceil(height);
  this->level=level;
  this->unit_width=this->width/(double)(1<<this->level);
  this->unit_height=this->height/(double)(1<<this->level);
}

void QuadTree::addRoute(const vector<array<double,2>> &route,const Ant *a){
  for(int i=0;i<route.size();i++){
    const array<double,2> &point=route[i];
    if(!inWorld(point[0],point[1]))continue;
    int morton_num=mortonNumber(point[0],point[1]);
    quad_tree[morton_num].insert({point,a});
  }
}

vector<pair<const array<double,2>&,const Ant*>>* QuadTree::reachablePoints(double cx,double cy,double width,double height){
  vector<pair<const array<double,2>&,const Ant*>>* ret=new vector<pair<const array<double,2>&,const Ant*>>;
  stack<array<uint,3>>s;//left_up_morton,right_down_morton,depth
  double hw=width/2,hh=height/2;
  if(!inWorld(cx-hw,cy-hh)||!inWorld(cx+hw,cy+hh))return NULL;//世界も矩形なのでこの二点の確認だけで良い
  const int MAX_DEPTH=ceil(log2(this->width*this->height/(width*height)));
  array<uint,3>tmp;
  tmp[0]=mortonNumber(cx-hw,cy-hh);tmp[1]=mortonNumber(cx+hw,cy+hh);
  tmp[2]=mortonDepth(tmp[0],tmp[1]);
  s.push(tmp);
  while(!s.empty()){
    tmp=s.top();s.pop();
    if(tmp[2]>=MAX_DEPTH){
      searchMorton(tmp[0],tmp[2],*ret);
      continue;
    }
    uint next_depth=tmp[2]+1;
    uint p1_masked_x=tmp[0]&MASK_X,p1_masked_y=tmp[0]&MASK_Y;
    uint p2_masked_x=tmp[1]&MASK_X,p2_masked_y=tmp[1]&MASK_Y;
    if((tmp[0]^tmp[1])>>(sizeof(uint)*8-(tmp[2]+1)*2)==0x3){//FIXME: ここ全体が未だ駄目。
      uint tmp_p;
      s.push({mortonInDepth(tmp[1],next_depth),tmp[1],next_depth});//11
      s.push({
        p1_masked_x^(MASK_X&(mortonInDepth(p2_masked_y,tmp[2])^((uint)0x2<<(sizeof(uint)-2*next_depth)))),
        p2_masked_y^(MASK_Y&(mortonInDepth(p1_masked_x,tmp[2])^((uint)0x2<<(sizeof(uint)-2*next_depth)^~(UINT32_MAX>>2*(next_depth+1))))),
        next_depth
      });//10
      s.push({
        p1_masked_y^(MASK_Y&(mortonInDepth(p2_masked_x,tmp[2])^((uint)0x1<<(sizeof(uint)-2*next_depth)))),
        p2_masked_x^(MASK_X&(mortonInDepth(p1_masked_y,tmp[2])^((uint)0x1<<(sizeof(uint)-2*next_depth)^~(UINT32_MAX>>2*(next_depth+1))))),
        next_depth
      });//01
      s.push({tmp[0],tmp[0]|(UINT32_MAX>>(next_depth+1)*2),next_depth});//00
    }else{
      s.push({tmp[0],p2_masked_y|mortonInDepth(tmp[0],next_depth),next_depth});
      s.push({tmp[0],tmp[0]|(UINT32_MAX>>(next_depth+1)*2),next_depth});
    }
  }
}

//以下でprivate関数

uint QuadTree::separate(ushort n) {
  uint lln=n,ret=0;
  for(int i=0;i<sizeof(n)*8/2;i++){
    ret+=(lln&(ushort)1<<i)<<i;
  }
  return ret;
}

bool QuadTree::inWorld(double x,double y){
  if(x>=0&&y>=0&&x<=this->width&&y<=this->height){
    fprintf(stderr,"QuadTree: Irregular point (%g, %g) was observed.\n",x,y);
    return false;
  }
  return true;
}

uint QuadTree::mortonNumber(double x, double y) {
  //x==this->widthになるとオーバーフローするのでその対策
  ushort i=(ushort)(min(x/this->unit_width,(double)(1<<this->level)));
  ushort j=(ushort)(min(y/this->unit_height,(double)(1<<this->level)));
  return (separate(i) | separate(j) << 1);
}

void QuadTree::searchMorton(uint morton,int search_depth,vector<pair<const array<double,2>&,const Ant*>> &buf){
  const uint LOWER_BIT_MAX=UINT32_MAX>>(2*search_depth),MORTON_HIGHER_BIT=morton&~LOWER_BIT_MAX,MORTON_MAX=MORTON_HIGHER_BIT+LOWER_BIT_MAX;
  auto e=this->quad_tree.lower_bound(MORTON_HIGHER_BIT);
  if(e->first<=MORTON_HIGHER_BIT)++e;
  for(;e->first<=MORTON_MAX;++e){
    for(const auto &g:e->second){//NOTE: 本当はsetをbufに詰められる様にした方が早そうだが、簡単なのでこうする
      buf.push_back(g);
    }
  }
}