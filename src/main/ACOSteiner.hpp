#ifndef _ACO_STEINER_HPP_
#define _ACO_STEINER_HPP_
#include<array>
using namespace std;
using ll =long long;

class Ant;
class ACOTable;
class QuadTree;

class ACOSteiner{
  protected:
    ll time=0;
    double min_space,max_space;
    vector<array<double,2>> points;
    array<double,2> points_offset;
    ACOTable* table;
    vector<QuadTree> qtworld;
  public:
    ACOSteiner(const vector<array<double,2>> &points);
    //~ACOSteiner();//TODO: 実装!
    void search();
    ll getTime() const;
    void setMinSpace(double d);
    void setMaxSpace(double d);
    double getMinSpace() const;
    double getMaxSpace() const;
    void setTableSize(ll size);
    const ACOTable* getACOTable() const;
};


#endif//_ACO_STEINER_HPP_