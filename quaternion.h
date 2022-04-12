#ifndef QUAT_H
#define QUAT_H
#include <iostream>
#include <cmath>
#include <initializer_list>
#include <array>
#include "vec3.h"

template <typename T>
class Quaternion_Base;
template <typename T>
class Quaternion;
template <typename T>
class Unit_Quaternion;

template <typename T>
class Quaternion_Base {
protected:
    T w, x, y, z;
public:
    Quaternion_Base() : w{0}, x{0}, y{0}, z{0} {}
    Quaternion_Base(const T& w, const T& x , const T& y, const T& z) : w{w}, x{x}, y{y}, z{z} {}

    virtual T                   operator[](const unsigned int i) const;
    virtual Quaternion_Base<T>  conjugated() const                  {return {w,-x,-y,-z};}
    virtual void                conjugate()                         {x *= -1; y *= -1; z *= -1;}
    virtual T                   real()                              {return w;}
    virtual Vec3<T>             imag()                              {return {x,y,z};}
};

template <typename T>
T Quaternion_Base<T>::operator[](const unsigned int i) const
{
    switch(i) {
        case 0: return this->w;
        case 1: return this->x;
        case 2: return this->y;
        case 3: return this->z;
        default: assert(0);
    };
    //Should never happen
    return 0;
}

template <typename T>
class Quaternion : public Quaternion_Base<T> {
public:
    Quaternion() : Quaternion_Base<T>{0,0,0,0} {}
    Quaternion(const T& w, const T& x , const T& y, const T& z) : Quaternion_Base<T>{w,x,y,z} {}
    Quaternion(const Unit_Quaternion<T>& q) : Quaternion_Base<T>{q} {}

    Quaternion<T>&      operator*=(const Quaternion<T>& q);
    Quaternion<T>&      operator+=(const Quaternion<T>& q)  {this->w += q.w; this->x += q.x; this->y += q.y; this->z += q.z; return *this;}
    Quaternion<T>&      operator-=(const Quaternion<T>& q)  {this->w -= q.w; this->x -= q.x; this->y -= q.y; this->z -= q.z; return *this;}
    Quaternion<T>&      operator/=(const T& a)              {this->w /= a; this->x /= a; this->y /= a; this->z /= a; return *this;}
    Quaternion<T>&      operator*=(const T& a)              {this->w *= a; this->x *= a; this->y *= a; this->z *= a; return *this;}
};
template <typename T>
Quaternion<T>& Quaternion<T>::operator*=(const Quaternion<T>& q)
{
    Quaternion<T> p = *this;
    this->w = p.w*q.w - p.x*q.x - p.y*q.y - p.z*q.z;
    this->x = p.w*q.x + q.w*p.x + (p.y*q.z - q.y*p.z);
    this->y = p.w*q.y + q.w*p.y - (p.x*q.z - q.x*p.z);
    this->z = p.w*q.z + q.w*p.z + (p.x*q.y - q.x*p.y);
    return *this;
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
    return {0,u[0],u[1],u[2]};
}


template <typename T>
class Unit_Quaternion : public Quaternion_Base<T> {
private:
    void normalize() {T norm = std::sqrt(this->w*this->w + this->x*this->x + this->y*this->y + this->z*this->z); this->w /= norm; this->x /= norm; this->y /= norm; this->z /= norm;}
public:
    Unit_Quaternion() : Quaternion_Base<T>{1,0,0,0} {}
    Unit_Quaternion(const T& w, const T& x, const T& y, const T& z) : Quaternion_Base<T>{w,x,y,z} {normalize();}
    Unit_Quaternion(const T& angle, const Vec3<T>& axis) {this->w = std::cos(angle/2);this->x = std::sin(angle/2)*axis[0];this->y = std::sin(angle/2)*axis[1];this->z = std::sin(angle/2)*axis[2];}

    Unit_Quaternion<T>&      operator*=(const Unit_Quaternion<T>& q);

};
template <typename T>
std::ostream& operator<<(std::ostream& os, Unit_Quaternion<T> q)
{
    return os << "Unit_Quaternion: " <<  "{" << q[0] << ", " << q[1] << ", " << q[2] << ", " << q[3] << "}";
}
template <typename T>
Unit_Quaternion<T>& Unit_Quaternion<T>::operator*=(const Unit_Quaternion<T>& q)
{
    Unit_Quaternion<T> p = *this;
    this->w = p.w*q.w - p.x*q.x - p.y*q.y - p.z*q.z;
    this->x = p.w*q.x + q.w*p.x + (p.y*q.z - q.y*p.z);
    this->y = p.w*q.y + q.w*p.y - (p.x*q.z - q.x*p.z);
    this->z = p.w*q.z + q.w*p.z + (p.x*q.y - q.x*p.y);
    return *this;
}
template <typename T>
Quaternion<T> operator*(const Quaternion<T>& q, const Unit_Quaternion<T>& p)
{
    Quaternion<T> qp{p};
    return q*qp;
}
template <typename T>
Quaternion<T> operator*(const Unit_Quaternion<T>& p, const Quaternion<T>& q)
{
    Quaternion<T> qp{p};
    return qp*q;
}
template <typename T>
Unit_Quaternion<T> operator*(const Unit_Quaternion<T>& p, const Unit_Quaternion<T>& q)
{
    Unit_Quaternion<T> res = p;
    res *= q;
    return res;
}
#endif
