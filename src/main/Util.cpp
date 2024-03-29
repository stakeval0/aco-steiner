#include "Util.hpp"

#include <cmath>

double euclid(const array<double, 2>& v) {
  return sqrt(v[0] * v[0] + v[1] * v[1]);
}

double euclid(const array<double, 2>& a, const array<double, 2>& b) {
  return euclid(a - b);
}

void rotate(array<double, 2>& v, double theta) {
  const auto V = v;
  v[0] = V[0] * cos(theta) - V[1] * sin(theta);
  v[1] = V[0] * sin(theta) + V[1] * cos(theta);
}

array<double, 2>& operator+=(array<double, 2>& v, double d) {
  v[0] += d;
  v[1] += d;
  return v;
}

array<double, 2>& operator+=(array<double, 2>& v1, const array<double, 2>& v2) {
  v1[0] += v2[0];
  v1[1] += v2[0];
  return v1;
}

array<double, 2>& operator-=(array<double, 2>& v, double d) {
  v[0] -= d;
  v[1] -= d;
  return v;
}

array<double, 2>& operator-=(array<double, 2>& v1, const array<double, 2>& v2) {
  v1[0] -= v2[0];
  v1[1] -= v2[0];
  return v1;
}

array<double, 2>& operator*=(array<double, 2>& v, double d) {
  v[0] *= d;
  v[1] *= d;
  return v;
}

array<double, 2>& operator*=(array<double, 2>& v1, const array<double, 2>& v2) {
  v1[0] *= v2[0];
  v1[1] *= v2[0];
  return v1;
}

array<double, 2>& operator/=(array<double, 2>& v, double d) {
  v[0] /= d;
  v[1] /= d;
  return v;
}

array<double, 2>& operator/=(array<double, 2>& v1, const array<double, 2>& v2) {
  v1[0] /= v2[0];
  v1[1] /= v2[0];
  return v1;
}

array<double, 2> operator+(const array<double, 2>& v, double d) {
  return {v[0] + d, v[1] + d};
}

array<double, 2> operator+(double d, const array<double, 2>& v) {
  return v + d;
}

array<double, 2> operator+(const array<double, 2>& v1,
                           const array<double, 2>& v2) {
  return {v1[0] + v2[0], v1[1] + v2[1]};
}

array<double, 2> operator-(const array<double, 2>& v, double d) {
  return {v[0] - d, v[1] - d};
}

array<double, 2> operator-(double d, const array<double, 2>& v) {
  return {d - v[0], d - v[1]};
}

array<double, 2> operator-(const array<double, 2>& v1,
                           const array<double, 2>& v2) {
  return {v1[0] - v2[0], v1[1] - v2[1]};
}

array<double, 2> operator*(const array<double, 2>& v, double d) {
  return {v[0] * d, v[1] * d};
}

array<double, 2> operator*(double d, const array<double, 2>& v) {
  return v * d;
}

array<double, 2> operator*(const array<double, 2>& v1,
                           const array<double, 2>& v2) {
  return {v1[0] * v2[0], v1[1] * v2[1]};
}

double dot(const array<double, 2>& v1, const array<double, 2>& v2) {
  double ret = 0;
  for (int i = 0; i < v1.size(); i++) ret += v1[i] * v2[i];
  return ret;
}

double cross(const array<double, 2>& v1, const array<double, 2>& v2) {
  return v1[0] * v2[1] - v1[1] * v2[0];
}

template <class T>
T mean(const T& a, const T& b) {
  return (a + b) / 2;
}

// NOTE: 出典はhttps://zenn.dev/ymd_h/articles/e90ad8ad40a6dd
template <typename T>
class isIterable {
 private:
  template <typename U>
  static constexpr auto ADL(U&& v)
      -> decltype(begin(v), end(v), std::true_type());
  static constexpr std::false_type ADL(...);

  template <typename U>
  static constexpr auto STD(U&& v)
      -> decltype(std::begin(v), std::end(v), std::true_type());
  static constexpr std::false_type STD(...);

  template <typename U>
  static constexpr auto Member(U&& v)
      -> decltype(v.begin(), v.end(), std::true_type());
  static constexpr std::false_type Member(...);

 public:
  static constexpr bool value = (decltype(ADL(std::declval<T>()))::value ||
                                 decltype(STD(std::declval<T>()))::value ||
                                 decltype(Member(std::declval<T>()))::value);
};

/* template<template<class> class T>
 double squareMean(const T<double> &x){

 } */
