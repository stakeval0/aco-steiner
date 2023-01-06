#ifndef _UTIL_HPP_
#define _UTIL_HPP_
#include<array>
using namespace std;

double euclid(const array<double,2> &a,const array<double,2> &b);
void rotate(array<double,2> &v,double theta);
array<double,2> operator+(const array<double,2> &v,double d);
array<double,2> operator+(double d,const array<double,2> &v);
array<double,2> operator+(const array<double,2> &v1,const array<double,2> &v2);
array<double,2> operator-(const array<double,2> &v,double d);
array<double,2> operator-(double d,const array<double,2> &v);
array<double,2> operator-(const array<double,2> &v1,const array<double,2> &v2);
array<double,2> operator*(const array<double,2> &v,double d);
array<double,2> operator*(double d,const array<double,2> &v);
array<double,2> operator*(const array<double,2> &v1,const array<double,2> &v2);
double dot(const array<double,2> &v1,const array<double,2> &v2);
double cross(const array<double,2> &v1,const array<double,2> &v2);

#endif//_UTIL_HPP_