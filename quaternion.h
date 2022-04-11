#ifndef QUAT_H
#define QUAT_H
#include <iostream>
#include <cmath>
#include <initializer_list>
#include "vec3.h"

template <typename T>
class Quaternion {
protected:
    T w,x,y,z;
public:
    Quaternion() {w = 0; x = 0; y = 0; z = 0;}
    Quaternion(const T w, const T x , const T y, const T z) : w{w}, x{x}, y{y}, z{z} {}
    Quaternion(std::initializer_list<T> l) = delete;
    Quaternion<T>& operator*=(const Quaternion<T>& q);
    Quaternion<T>& operator+=(const Quaternion<T>& q) {w += q.w; x += q.x; y += q.y; z += q.z; return *this;}
    Quaternion<T>& operator-=(const Quaternion<T>& q) {w -= q.w; x -= q.x; y -= q.y; z -= q.z; return *this;}
    Quaternion<T>& operator/=(const T& a) {w /= a; x /= a; y /= a; z /= a; return *this;}
    Quaternion<T>& operator*=(const T& a) {w *= a; x *= a; y *= a; z *= a; return *this;}
    T operator[](const unsigned int i) const;
    Quaternion<T> conjugated() const {return {w,-x,-y,-z};}
    void conjugate() {x *= -1; y *= -1; z *= -1;}
    T real() {return w;}
    Vec3<T> imag() {return {x,y,z};}

};
template <typename T>
Quaternion<T>& Quaternion<T>::operator*=(const Quaternion<T>& q)
{
    Quaternion<T> p = *this;
    w = p.w*q.w - p.x*q.x - p.y*q.y - p.z*q.z;
    x = p.w*q.x + q.w*p.x + (p.y*q.z - q.y*p.z);
    y = p.w*q.y + q.w*p.y - (p.x*q.z - q.x*p.z);
    z = p.w*q.z + q.w*p.z + (p.x*q.y - q.x*p.y);
    return *this;
}
template <typename T>
T Quaternion<T>::operator[](const unsigned int i) const
{
    T res = 0;
    switch(i) {
        case 0:
            res = w;
            break;
        case 1:
            res = x;
            break;
        case 2:
            res = y;
            break;
        case 3:
            res = z;
            break;
        default:
            assert(0);
            break;
    };
    return res;
}
template <typename T>
std::ostream& operator<<(std::ostream& os, const Quaternion<T>& q)
{
    return os << "Quaternion: " <<  "{" << q[0] << ", " << q[1] << ", " << q[2] << ", " << q[3] << "}";
}

template <typename T>
Quaternion<T> operator+(const Quaternion<T>& q, const Quaternion<T>& p)
{
    Quaternion<T> res = q;
    res += p;
    return res;
}
template <typename T>
Quaternion<T> operator-(const Quaternion<T>& q, const Quaternion<T>& p)
{
    Quaternion<T> res = q;
    res -= p;
    return res;
}
template <typename T>
Quaternion<T> operator*(const Quaternion<T>& q, const Quaternion<T>& p)
{
    Quaternion<T> res = q;
    res *= p;
    return res;
}
template <typename T>
Quaternion<T> operator/(const Quaternion<T>& q, const Quaternion<T>& p)
{
    Quaternion<T> res = q;
    res *= p.conjugated();
    return res;
}
template <typename T>
Quaternion<T> operator-(const Quaternion<T>& q)
{
    return {-q[0],-q[1],-q[2],-q[3]};
}
template <typename T>
Quaternion<T> operator*(const T& a, const Quaternion<T>& q)
{
    Quaternion<T> res = q;
    res *= a;
    return res;
}
template <typename T>
Quaternion<T> operator*(const Quaternion<T>& q, const T& a)
{
    Quaternion<T> res = q;
    res *= a;
    return res;
}
template <typename T>
Quaternion<T> operator/(const T& a, const Quaternion<T>& q)
{
    Quaternion<T> res = q;
    res /= a;
    return res;
}
template <typename T>
Quaternion<T> operator/(const Quaternion<T>& q, const T& a)
{
    Quaternion<T> res = q;
    res /= a;
    return res;
}
template <typename T>
Vec3<T> quat_to_vec(const Quaternion<T>& q)
{
    return {q[1],q[2],q[3]};
}
template <typename T>
Quaternion<T> vec_to_quat(const Vec3<T>& u)
{
    return Quaternion<T>(0,u[0],u[1],u[2]);
}
template <typename T>
class Unit_Quaternion : public Quaternion<T> {
public:
    Unit_Quaternion() : Quaternion<T>(1,0,0,0) {}
    Unit_Quaternion(T w, T x, T y, T z) : Quaternion<T>(w,x,y,z) {}
    Unit_Quaternion(T angle, Vec3<T> axis) {this->w = std::cos(angle/2);this->x = std::sin(angle/2)*axis[0];this->y = std::sin(angle/2)*axis[1];this->z = std::sin(angle/2)*axis[2];}
    Unit_Quaternion(std::initializer_list<T> l) = delete;
    void normalize() {T norm = std::sqrt(this->w*this->w + this->x*this->x + this->y*this->y + this->z*this->z); *this /= norm;}
};
template <typename T>
std::ostream& operator<<(std::ostream& os, Unit_Quaternion<T> q)
{
    return os << "Unit_Quaternion: " <<  "{" << q[0] << ", " << q[1] << ", " << q[2] << ", " << q[3] << "}";
}
template <typename T>
Quaternion<T> operator*(const Quaternion<T>& q, const Unit_Quaternion<T>& p)
{
    return q*static_cast<Quaternion<T>>(p);
}
template <typename T>
Quaternion<T> operator*(const Unit_Quaternion<T>& p, const Quaternion<T>& q)
{
    return q*static_cast<Quaternion<T>>(p);
}
template <typename T>
Unit_Quaternion<T> operator*(const Unit_Quaternion<T>& p, const Unit_Quaternion<T>& q)
{
    Unit_Quaternion<T> res = p;
    res *= q;
    return res;
}
#endif
