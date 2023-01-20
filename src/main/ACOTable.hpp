#ifndef _ACO_TABLE_HPP_
#define _ACO_TABLE_HPP_
#include <array>
#include <functional>
#include <set>
// #include"../tools/JsonHead.hpp"
using namespace std;

class Ant;

enum class ACOTableColumn {
  COST,
  LENGTH,
  PHEROMONE,
};

// 最良と最悪どちらも取り出したいのでmultiset方式
class ACOTable
    : public multiset<const Ant*, function<bool(const Ant*, const Ant*)>> {
 private:
  long long max_size = 16;

 public:
  ACOTable();
  inline void setCapacity(long long size) { this->max_size = size; };
  inline long long getCapacity() const { return this->max_size; };
  const Ant* dropout();
  double sum(ACOTableColumn target) const;
  double mean(ACOTableColumn target) const;
  double variance(ACOTableColumn target) const;
  double stdev(ACOTableColumn target) const;
  double best(ACOTableColumn target) const;
  string json() const;
  string json(const int begin, const int end) const;
  // json getElementJson(const int index) const;
  // double pheromoneSum()
  // const;//出来ればupdatePheromone()時に計算して変数として保存しておきたい
};

#endif  //_ACO_TABLE_HPP_