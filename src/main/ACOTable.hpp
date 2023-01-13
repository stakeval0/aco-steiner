#ifndef _ACO_TABLE_HPP_
#define _ACO_TABLE_HPP_
#include<array>
#include<set>
#include<functional>
using namespace std;
using ll=long long;

class Ant;

enum class ACOTableColumn{
  COST,
  LENGTH,
  PHEROMONE,
};

//最良と最悪どちらも取り出したいのでmultiset方式
class ACOTable : public multiset<const Ant*,function<bool(const Ant*,const Ant*)>> {
  private:
    ll max_size=16;
  public:
    ACOTable();
    void setCapacity(ll size);
    ll getCapacity() const;
    const Ant* dropout();
    double sum(ACOTableColumn target) const;
    double mean(ACOTableColumn target) const;
    double variance(ACOTableColumn target) const;
    double stdev(ACOTableColumn target) const;
    //double pheromoneSum() const;//出来ればupdatePheromone()時に計算して変数として保存しておきたい
};

#endif//_ACO_TABLE_HPP_