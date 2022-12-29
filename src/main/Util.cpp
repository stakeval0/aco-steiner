#include<cmath>
#include"Util.hpp"

double euclid(const array<double,2> &a,const array<double,2> &b){
  double dx=b[0]-a[0],dy=b[1]-a[1];
  return sqrt(dx*dx+dy*dy);
}

//NOTE: 出典はhttps://zenn.dev/ymd_h/articles/e90ad8ad40a6dd
template<typename T> class isIterable {
  private:
    template<typename U>
    static constexpr auto ADL(U&& v)
      -> decltype(begin(v), end(v), std::true_type());
    static constexpr std::false_type ADL(...);

    template<typename U>
    static constexpr auto STD(U&& v)
      -> decltype(std::begin(v), std::end(v), std::true_type());
    static constexpr std::false_type STD(...);

    template<typename U>
    static constexpr auto Member(U&& v)
      -> decltype(v.begin(), v.end(), std::true_type());
    static constexpr std::false_type Member(...);
  public:
    static constexpr bool value = (
      decltype(ADL(std::declval<T>()))::value ||
  		decltype(STD(std::declval<T>()))::value ||
  		decltype(Member(std::declval<T>()))::value
    );
};

/* template<template<class> class T>
 double squareMean(const T<double> &x){
   
 } */
