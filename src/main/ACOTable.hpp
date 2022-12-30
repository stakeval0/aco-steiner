#ifndef _ACO_TABLE_HPP_
#define _ACO_TABLE_HPP_
#include<array>
#include<set>
#include<functional>
using namespace std;
using ll=long long;

class Ant;

//最良と最悪どちらも取り出したいのでmultiset方式
class ACOTable : public multiset<const Ant*,function<bool(const Ant*,const Ant*)>> {
  private:
    ll max_size=16;
  public:
    ACOTable();
    void setCapacity(ll size);
    ll getCapacity() const;
    double costVariance() const;
    const Ant* dropout();
    //double pheromoneSum() const;//出来ればupdatePheromone()時に計算して変数として保存しておきたい
};

#endif//_ACO_TABLE_HPP_