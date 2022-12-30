#ifndef _ACO_STEINER_HPP_
#define _ACO_STEINER_HPP_
#include<array>
#include<functional>
using namespace std;
using ll =long long;

class Ant;
class ACOTable;
class QuadTree;

class ACOSteiner{
  protected:
    ll time=0;
    double min_distance,max_distance;
    double mutation_probability=0.1,evaporation_cofficient=0.9,basic_move_ratio=0.1;
    double pheromone_cofficient;
    vector<array<double,2>> points;//NOTE: constは掛けられないが実質const
    array<double,2> points_offset;
    ACOTable *table;
    vector<QuadTree> qtworld;
    function<double(const array<double,2> &,const array<double,2> &)> cost_function;
    void countTime();
  public:
    ACOSteiner(const vector<array<double,2>> &points);
    ~ACOSteiner();
    void search();
    ll getTime() const;
    const vector<array<double,2>>& getConnectPoints() const;
    const array<double,2>& getConnectPoint(int index) const;
    void setCostFunction(const function<double(const array<double,2> &,const array<double,2> &)> &f);
    double calcCost(const array<double,2> &,const array<double,2> &) const;
    void setMinDistance(double d);
    void setMaxDistance(double d);
    void setMutationProbability(double value);
    void setEvaporationCofficient(double value);
    void setBasicMoveRatio(double value);
    void setTableCapacity(ll size);
    double getMinDistance() const;
    double getMaxDistance() const;
    double getMutationProbability() const;
    double getEvaporationCofficient() const;
    double getBasicMoveRatio() const;
    ll getTableCapacity() const;
    const ACOTable& getACOTable() const;
    const QuadTree& getQuadTree(int index) const;
};

#endif//_ACO_STEINER_HPP_