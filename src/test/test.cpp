#include<iostream>
#include<vector>
#include"../main/QuadTree.hpp"
using namespace std;

void push(vector<vector<int>> &v,int begin){
  vector<int>tmp;
  for(int i=0;i<10;i++)tmp.push_back(i+begin);
  v.push_back(tmp);
}

int main(void){
  vector<vector<int>>v;
  for(int i=0;i<5;i++)push(v,i);
  for(int i=0;i<v.size();i++)
    for(int j=0;j<v[i].size();j++)
      cout<<v[i][j]<<(j!=v[i].size()-1?' ':'\n');
  return 0;
}