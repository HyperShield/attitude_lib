#ifndef VEC_H
#define VEC_H
#include <iostream>
#include <cmath>
#include <initializer_list>
#include <cassert>
#include "mat3.h"

template <typename T>
class Vec3 {
private:
    T x,y,z;
public:
    Vec3() : x{0}, y{0}, z{0} {}
    Vec3(const T& x, const T& y, const T& z) : x{x}, y{y}, z{z} {}

    T           operator[](const unsigned int i) const;
    Vec3<T>&    operator+=(const Vec3<T>& v)            {x += v.x; y += v.y; z += v.z; return *this;}
    Vec3<T>&    operator-=(const Vec3<T>& v)            {x -= v.x; y -= v.y; z -= v.z; return *this;}
    Vec3<T>&    operator*=(const T& a)                  {x *= a; y *= a; z *= a; return *this;}
    Vec3<T>&    operator/=(const T& a)                  {x /= a; y /= a; z /= a; return *this;}
    T magnitude() {return std::sqrt(x*x + y*y + z*z);}
};
template <typename T>
T Vec3<T>::operator[](const unsigned int i) const
{
    switch(i) {
    case 0: return x;
    case 1: return y;
    case 2: return z;
    default: assert(0);
    }
    //Should never happen
    return 0;
}

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
template <typename T>
Mat3<T> outer(Vec3<T> u, Vec3<T> v)
{
    return {{u[0]*v[0],u[0]*v[1],u[0]*v[2]},{u[1]*v[0],u[1]*v[1],u[1]*v[2]},{u[2]*v[0],u[2]*v[1],u[2]*v[2]}};
}
template <typename T>
Mat3<T> skew(Vec3<T> u)
{
    return {{0,-u[2],u[1]},{u[2],0,-u[0]},{-u[1],u[0],0}};
}
template <typename T>
Vec3<T> vex(Mat3<T> M)
{
    return {M(2,1),M(3,1),M(3,2)};
}
#endif
