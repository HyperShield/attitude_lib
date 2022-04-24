#ifndef ATT_H
#define ATT_H
#include <iostream>
#include <cmath>
#include <initializer_list>
#include "../inc/quaternion.h"
#include "../inc/vec3.h"
#include "../inc/mat3.h"

template <typename T>
class attitude {
private:
    Unit_Quaternion<T>  q;
    Unit_Quaternion<T>  integrate_euler(const Unit_Quaternion<T>& q, const Vec3<T>& w, const T& dt);
    Unit_Quaternion<T>  integrate_rk2(const Unit_Quaternion<T>& q, const Vec3<T>& w, const T& dt);
    Unit_Quaternion<T>  integrate_rk4(const Unit_Quaternion<T>& q, const Vec3<T>& w, const T& dt);
    Quaternion<T>       attitude_kinematics(const Unit_Quaternion<T>& q, const Vec3<T>& w);
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
    q = integrate_euler(q, w, dt);
}
template <typename T>
Quaternion<T> attitude<T>::attitude_kinematics(const Unit_Quaternion<T>& q, const Vec3<T>& w)
{
    Quaternion<T> qw{w};
    return static_cast<T>(1)/2*(q*qw);
}
template <typename T>
Unit_Quaternion<T> attitude<T>::integrate_euler(const Unit_Quaternion<T>& q, const Vec3<T>& w, const T& dt)
{
    Quaternion<T> p{q};
    Quaternion<T> dot_q = attitude_kinematics(q, w);
    p += dot_q*dt;
    return {p};
}
template <typename T>
Unit_Quaternion<T> attitude<T>::integrate_rk2(const Unit_Quaternion<T>& q, const Vec3<T>& w, const T& dt)
{
    auto f1 = attitude_kinematics(q, w);
    auto k1 = Quaternion<T>{q} + f1*(dt/2);

    auto f2 = attitude_kinematics({k1}, w);

    auto res = Quaternion<T>{q} + f2*dt;
    return {res};
}
template <typename T>
Unit_Quaternion<T> attitude<T>::integrate_rk4(const Unit_Quaternion<T>& q, const Vec3<T>& w, const T& dt)
{
    auto f1 = attitude_kinematics(q, w);
    auto k1 = Quaternion<T>{q} + f1*(dt/2);

    auto f2 = attitude_kinematics({k1}, w);
    auto k2 = Quaternion<T>{q} + f2*(dt/2);

    auto f3 = attitude_kinematics({k2}, w);
    auto k3 = Quaternion<T>{q} + f3*dt;

    auto f4 = attitude_kinematics({k3}, w);

    auto res = Quaternion<T>{q} + (dt/6)*(f1 + f4) + (dt/3)*(f2 + f3);
    return {res};
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
