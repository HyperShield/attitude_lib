#ifndef ATT_H
#define ATT_H
#include <iostream>
#include <cmath>
#include <initializer_list>
#include "quaternion.h"
#include "vec3.h"
#include "mat3.h"

template <typename T>
class attitude {
private:
    Unit_Quaternion<T> q;
    Unit_Quaternion<T> integrate(const Unit_Quaternion<T>& q, const Quaternion<T>& dot_q, const T& dt);
public:
    attitude() : q{1,0,0,0} {}
    attitude(const T& w, const T& x, const T& y, const T& z) : q{w,x,y,z} {}

    void                update_attitude(const Vec3<T>& w, const T& dt);
    Vec3<T>             get_attitude_euler();
    Mat3<T>             get_attitude_dcm();
    Unit_Quaternion<T>  get_attitude_quaternion();
    void                set_attitude(const Vec3<T>& euler);
    void                set_attitude(const Mat3<T>& DCM);
    void                set_attitude(const Unit_Quaternion<T>& q);
    void                set_attitude(const T& ang, const Vec3<T>& axis);
};
template <typename T>
void attitude<T>::update_attitude(const Vec3<T>& w, const T& dt)
{
    auto dot_q = static_cast<T>(0.5)*(q*vec_to_quat(w));
    q = integrate(q,dot_q,dt);
}
template <typename T>
Unit_Quaternion<T> attitude<T>::integrate(const Unit_Quaternion<T>& q, const Quaternion<T>& dot_q, const T& dt)
{
    Quaternion<T> p{q};
    p += dot_q*dt;
    return {p[0],p[1],p[2],p[3]};
}
template <typename T>
Vec3<T> attitude<T>::get_attitude_euler()
{
    Vec3<T> E{std::atan2(2*(q[0]*q[1] + q[2]*q[3]), q[0]*q[0] + q[3]*q[3] - q[1]*q[1] - q[2]*q[2]),
              std::asin(2*(q[0]*q[2] - q[1]*q[3])),
              std::atan2(2*(q[0]*q[3] + q[1]*q[2]), q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3])};
    return E;
}
template <typename T>
Unit_Quaternion<T> attitude<T>::get_attitude_quaternion()
{
    return q;
}
template <typename T>
void attitude<T>::set_attitude(const Unit_Quaternion<T>& q)
{
    this->q = q;
}
template <typename T>
void attitude<T>::set_attitude(const T& ang, const Vec3<T>& axis)
{
    Unit_Quaternion<T> p{std::cos(ang/2),axis[0]*std::sin(ang/2),axis[1]*std::sin(ang/2),axis[2]*std::sin(ang/2)};
    q = p;
}
template <typename T>
void attitude<T>::set_attitude(const Vec3<T>& E)
{
    Unit_Quaternion<T> qx{std::cos(E[0]/2),std::sin(E[0]/2),0,0};
    Unit_Quaternion<T> qy{std::cos(E[1]/2),0,std::sin(E[1]/2),0};
    Unit_Quaternion<T> qz{std::cos(E[2]/2),0,0,std::sin(E[2]/2)};
    q = qz*qy*qx;
}
#endif
