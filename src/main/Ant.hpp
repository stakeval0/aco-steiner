#ifndef _ANT_HPP_
#define _ANT_HPP_
#include<vector>
#include<array>
#include<memory>
#include<functional>
#include<optional>
using namespace std;
using ll=long long;
using v2d=array<double,2>;//TODO: 後でクラスを作り直してリファクタリングしたい

class ACOSteiner;
template<class T> class QuadTree;
template<class T> struct QuadTreeNode;
struct Joint;
struct SingleRoute;

class Ant{
  protected:
    const ll BIRTH_TIME;
    const double MIN_DISTANCE,MAX_DISTANCE;
    double total_cost,total_length;//constは付けられないが、生成以降、少なくともpublic関数では定数として扱って良い
    mutable double pheromone_v;
    vector<shared_ptr<SingleRoute>>path;//pair<int,vec>のintで戻るノード数を管理する
    void tracePushBack(
        const int target_index,const v2d &e,
        const function<double(const v2d&,const v2d&)> &cost_function,
        QuadTree<const int> &own_qt);
    void properPushBack(
        const int target_index,const v2d &e,
        const function<double(const v2d&,const v2d&)> &cost_function,
        const v2d &min_point,const v2d &size);
    void properPushBack(
        const int target_index,const v2d &e,
        const function<double(const v2d&,const v2d&)> &cost_function,
        QuadTree<const int> &own_qt);
    template<class T>
    void joinCloseToNearestNode(
      const int target_index,const QuadTreeNode<T> &nearest_node,
      const function<double(const v2d&,const v2d&)> &cost_function,
      QuadTree<const int> &own_qt);
    const Ant* joinToOwn(
        const int route_index,const int index_in_route,QuadTree<const int> &own_qt,
        const double reachable_radius,
        const function<double(const v2d&,const v2d&)> &cost_function);
    const Ant* judgeJoinTo(
        const int route_index,const int first_index_in_route,const Ant* base_ant,
        const QuadTree<const Ant*> &QTA,QuadTree<const int> &own_qt,
        const double reachable_radius,
        const function<double(const v2d&,const v2d&)> &cost_function);
    pair<const Ant*,const array<int,2>> archedAdd(const int target_index,const Ant *base_ant,
                                    v2d base_random_vec,const ACOSteiner &world,
                                    QuadTree<const int> &own_qt);
    pair<const Ant*,const array<int,2>> addRandVecToOneRoute(const ACOSteiner &w,const Ant *base_ant,
                                     int index,QuadTree<const int> &own_qt);
    void joinToAnotherAnt(const Ant* base_ant);
    void joinToAnotherOwnRoute(const Ant* base_ant);
    void constructModifiedRoute(const ACOSteiner &w,ll current_time);
    void constructFirstRoute(const ACOSteiner &w);
  public:
    Ant(const ACOSteiner &world);
    void evaporate(double evaporation_cofficient) const;
    double pheromone() const;
    double cost() const;
    double length() const;
    const vector<v2d>& getRoute(int index) const;
    int routeNum() const;
};

#endif//_ANT_HPP_