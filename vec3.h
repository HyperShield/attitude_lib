#ifndef VEC_H
#define VEC_H
#include <iostream>
#include <cmath>
#include <initializer_list>
#include <cassert>

template <typename T>
class Vec3 {
private:
    T x,y,z;
public:
    Vec3() : x{0}, y{0}, z{0} {}
    Vec3(T x, T y, T z) : x{x}, y{y}, z{z} {}
    Vec3(std::initializer_list<T> l) = delete;
    T operator[](const unsigned int i) const {T res = 0; switch(i) { case 0: res = x; break; case 1: res = y; break; case 2: res = z; break; default: assert(0); }return res;}
    Vec3<T>& operator+=(const Vec3<T>& v) {x += v.x; y += v.y; z += v.z; return *this;}
    Vec3<T>& operator-=(const Vec3<T>& v) {x -= v.x; y -= v.y; z -= v.z; return *this;}
    Vec3<T>& operator*=(const T& a) {x *= a; y *= a; z *= a; return *this;}
    Vec3<T>& operator/=(const T& a) {x /= a; y /= a; z /= a; return *this;}
};
template <typename T>
T dot(const Vec3<T>& u, const Vec3<T>& v)
{
    return u[0]*v[0] + u[1]*v[1] + u[2]*v[2];
}
template <typename T>
Vec3<T> cross(const Vec3<T>& u, const Vec3<T>& v)
{
    return {u[1]*v[2] - v[1]*u[2], v[0]*u[2] - u[0]*v[2], u[0]*v[1] - v[0]*u[1]};
}
template <typename T>
Vec3<T> operator+(const Vec3<T>& u, const Vec3<T>& v)
{
    Vec3<T> res = u;
    res += v;
    return res;
}
template <typename T>
Vec3<T> operator-(const Vec3<T>& u, const Vec3<T>& v)
{
    Vec3<T> res = u;
    res -= v;
    return res;
}
template <typename T>
Vec3<T> operator*(const Vec3<T>& u, const T& a)
{
    Vec3<T> res = u;
    res *= a;
    return res;
}
template <typename T>
Vec3<T> operator*(const T& a, const Vec3<T>& u)
{
    Vec3<T> res = u;
    res *= a;
    return res;
}
template <typename T>
Vec3<T> operator/(const Vec3<T>& u, const T& a)
{
    Vec3<T> res = u;
    res /= a;
    return res;
}
template <typename T>
Vec3<T> operator/(const T& a, const Vec3<T>& u)
{
    Vec3<T> res = u;
    res /= a;
    return res;
}
template <typename T>
std::ostream& operator<<(std::ostream& os, const Vec3<T>& v)
{
    return os << "Vector<3>: " << "{" << v[0] << ", " << v[1] << ", " << v[2] << "}";
}
#endif
