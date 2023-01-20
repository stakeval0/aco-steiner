#ifndef _ACO_STEINER_HPP_
#define _ACO_STEINER_HPP_
#include <array>
#include <functional>
using namespace std;
using ll = long long;

class Ant;
class ACOTable;
template <class T>
class QuadTree;

using QuadTreeAnt = QuadTree<const Ant *>;

class ACOSteiner {
 protected:
  ll time = 0;
  double min_distance, max_distance;
  double mutation_probability = 0.1, evaporation_cofficient = 0.9,
         basic_move_ratio = 0.1;
  double pheromone_cofficient;
  vector<array<double, 2>> points;  // NOTE: constは掛けられないが実質const
  ACOTable *table;
  vector<QuadTreeAnt> qtworld;
  function<double(const array<double, 2> &, const array<double, 2> &)>
      cost_function;
  void countTime();

 public:
  ACOSteiner(const vector<array<double, 2>> &points);
  ~ACOSteiner();
  void search();
  inline ll getTime() const { return this->time; };
  inline const vector<array<double, 2>> &getConnectPoints() const {
    return this->points;
  };
  inline const array<double, 2> &getConnectPoint(int index) const {
    return this->points[index];
  };
  inline void setCostFunction(
      const function<double(const array<double, 2> &, const array<double, 2> &)>
          &f) {
    this->cost_function = f;
  };
  inline double calcCost(const array<double, 2> &p1,
                         const array<double, 2> &p2) const {
    return this->cost_function(p1, p2);
  };
  void setMinDistance(double d);
  void setMaxDistance(double d);
  void setMutationProbability(double value);
  void setEvaporationCofficient(double value);
  void setBasicMoveRatio(double value);
  void setTableCapacity(ll size);
  inline double getMinDistance() const { return this->min_distance; };
  inline double getMaxDistance() const { return this->max_distance; };
  inline double getMutationProbability() const {
    return this->mutation_probability;
  };
  inline double getEvaporationCofficient() const {
    return this->evaporation_cofficient;
  };
  inline double getBasicMoveRatio() const { return this->basic_move_ratio; };
  inline double getPheromoneCofficient() const {
    return this->pheromone_cofficient;
  };
  ll getTableCapacity() const;
  inline const ACOTable &getACOTable() const { return *this->table; };
  inline const vector<QuadTreeAnt> &getQuadTreeAntV() const {
    return this->qtworld;
  };
  const QuadTreeAnt &getQuadTreeAnt(int index) const;
};

#endif  //_ACO_STEINER_HPP_