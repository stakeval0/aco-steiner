#include<iostream>
#include<stack>
#include<cmath>
#include"QuadTree.hpp"

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
  uint direct_mask=x*QuadTree::MASK_X^y*QuadTree::MASK_Y;//それぞれの軸方向どちらに寄るか
  return (morton&~(mask_xy>>fixed_depth*2))^(direct_mask&mask_xy>>2*fixed_depth);
}

static void pushNewMortonArea(stack<array<uint,3>> &s,uint left_up_morton,uint right_bottom_morton){
  array<uint,3>tmp;
  tmp[0]=left_up_morton;tmp[1]=right_bottom_morton;
  tmp[2]=mortonDepth(tmp[0],tmp[1]);
  s.push(tmp);
}

//TODO: 後でlevelを消す
// 4分木のモデル
QuadTree::QuadTree(double width, double height, int level) {
  if(level>sizeof(ushort)*8){fprintf(stderr,"QuadTree: level is too big.\n");exit(EXIT_FAILURE);}
  this->width=width;
  this->height=height;
  this->level=level;
  this->unit_width=this->width/(double)(1<<this->level);
  this->unit_height=this->height/(double)(1<<this->level);
}

QuadTree::QuadTree(double width,double height){
  new (this) QuadTree(width,height,sizeof(uint)*8/2);
}

void QuadTree::addRoute(const vector<array<double,2>> &route,const Ant *a){
  for(int i=0;i<route.size();i++){
    const array<double,2> &point=route[i];
    if(!inWorld(point[0],point[1]))continue;
    int morton_num=mortonNumber(point[0],point[1]);
    quad_tree[morton_num].emplace(&point,a);
  }
}

void QuadTree::removeRoute(const vector<array<double,2>> &route,const Ant *a){
  for(int i=0;i<route.size();i++){
    const array<double,2> &point=route[i];
    if(!inWorld(point[0],point[1]))continue;
    int morton_num=mortonNumber(point[0],point[1]);
    quad_tree[morton_num].erase({&point,a});
  }
}

vector<pair<const array<double,2>&,const Ant*>>* QuadTree::reachablePoints(double cx,double cy,double width,double height){
  vector<pair<const array<double,2>&,const Ant*>>* ret=new vector<pair<const array<double,2>&,const Ant*>>;
  stack<array<uint,3>>s;//left_up_morton,right_down_morton,depth
  double hw=width/2,hh=height/2;
  double p1_x=max(cx-hw,0.),p1_y=max(cy-hh,0.),
         p2_x=min(cx+hw,this->width),p2_y=min(cy+hh,this->height);
  const int MAX_DEPTH=ceil(log2(this->width*this->height/(width*height))/2);
  array<uint,3>tmp;
  tmp[0]=mortonNumber(p1_x,p1_y);tmp[1]=mortonNumber(p2_x,p2_y);
  tmp[2]=mortonDepth(tmp[0],tmp[1]);
  s.push(tmp);
  while(!s.empty()){
    tmp=s.top();s.pop();
    if(tmp[2]>=MAX_DEPTH){
      searchMorton(tmp[0],tmp[2],*ret);
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

//以下でprivate関数

uint QuadTree::separate(ushort n) {
  uint lln=n,ret=0;
  for(int i=0;i<sizeof(n)*8;i++){
    ret+=(lln&((uint)1<<i))<<i;
  }
  return ret;
}

bool QuadTree::inWorld(double x,double y){
  if(x<0||y<0||x>this->width||y>this->height){
    fprintf(stderr,"QuadTree: Irregular point (%g, %g) was observed.\n",x,y);
    return false;
  }
  return true;
}

uint QuadTree::mortonNumber(double x, double y) {
  //x==this->widthになるとオーバーフローするのでその対策
  ushort i=min((int)(x/this->unit_width),(1<<this->level)-1);
  ushort j=min((int)(y/this->unit_height),(1<<this->level)-1);
  uint ret=(separate(i) | separate(j) << 1);
  return ret;
}

void QuadTree::searchMorton(uint morton,int search_depth,vector<pair<const array<double,2>&,const Ant*>> &buf){
  const uint LOWER_BIT_MAX=(search_depth<16?__UINT32_MAX__>>(2*search_depth):0),//ここを3項演算子でない状態でやっていたら何故か値がsearch_depth==16で__UINT32_MAX__になった
             MORTON_HIGHER_BIT=morton&~LOWER_BIT_MAX,MORTON_MAX=MORTON_HIGHER_BIT+LOWER_BIT_MAX;
  auto e=this->quad_tree.lower_bound(MORTON_HIGHER_BIT);
  //cout<<MORTON_HIGHER_BIT<<":\n";
  if(e!=this->quad_tree.end()&&e->first<MORTON_HIGHER_BIT)e++;
  for(;e!=this->quad_tree.end()&&e->first<=MORTON_MAX;e++){
    //cout<<e->first<<endl;
    for(const auto &g:e->second){//NOTE: 本当はsetをbufに詰められる様にした方が速そうだが、簡単なのでこうする
      buf.emplace_back(*(g.first),g.second);
    }
  }
}