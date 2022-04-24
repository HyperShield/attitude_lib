#ifndef EXPLICIT_COMPLEMENTARY_FILTER_H
#define EXPLICIT_COMPLEMENTARY_FILTER_H

#include <vector>
#include "quaternion.h"
#include "vec3.h"
#include "mat3.h"
/*
 * Constructor
 * Reset filter
 * Update filter
 * get state
 */

template <typename T, int N>
class ECF {
private:
    Unit_Quaternion<T> q;
    Vec3<T> b;
    std::vector<Vec3<T>> V;
    std::vector<Vec3<T>> U;
    std::vector<T> K;
    T ki;
    T kp;
    unsigned int i;
public:
    ECF() : q{1,0,0,0}, b{0,0,0}, V(N,{0,0,0}), U(N,{0,0,0}), K{N,0}, ki{0}, kp{0}, i{0} {}
    //template <typename... Tail>
    //ECF(Vec3<T> v, Tail... tail);
    Unit_Quaternion<T> get_attitude() {return q;}
    Vec3<T> get_bias() {return b;}

    ECF(ECF<T,N>& f) = delete;
    ECF<T,N>& operator=(ECF<T,N>& f) = delete;
    ECF(ECF<T,N>&& f) = delete;
    ECF<T,N>& operator=(ECF<T,N>&& f) = delete;

    template <typename... Tail>
    void set_gains(T kp, T ki, Tail... tail);
    void reset_filter() {q = {1,0,0,0}; b = {0,0,0};}
    template <typename... Tail>
    void set_reference_vectors(Vec3<T> v, Tail... tail);
    template <typename... Tail>
    void set_reference_vectors() {i = 0;}
    template <typename... Tail>
    void update_filter(Vec3<T> w, T dt, Tail... tail);
private:
    template <typename... Tail>
    void set_Ks(T k, Tail... tail);
    void set_Ks() {i = 0;}
    template <typename... Tail>
    void set_observation_vectors(Vec3<T> v, Tail... tail);
    template <typename... Tail>
    void set_observation_vectors() {i = 0;}
    void update_attitude(const Vec3<T>& w, const T& dt);
    Quaternion<T> attitude_kinematics(const Unit_Quaternion<T>& q, const Vec3<T>& w);
    Unit_Quaternion<T> integrate_euler(const Unit_Quaternion<T>& q, const Vec3<T>& w, const T& dt);
};
template <typename T, int N>
template <typename... Tail>
void ECF<T,N>::set_gains(T kp, T ki, Tail... tail)
{
    this->ki = ki;
    this->kp = kp;
    i = 0;
    set_Ks(tail...);
}
template<typename T, int N>
template <typename... Tail>
void ECF<T,N>::set_Ks(T k, Tail... tail)
{
    assert(i<N);
    K[i++] = k;
    set_Ks(tail...);
}
template <typename T, int N>
template <typename... Tail>
void ECF<T,N>::set_reference_vectors(Vec3<T> v, Tail... tail)
{
    assert(i < N);
    V[i++] = v;
    set_reference_vectors(tail...);
}
template <typename T, int N>
template <typename... Tail>
void ECF<T,N>::set_observation_vectors(Vec3<T> u, Tail... tail)
{
    assert(i < N);
    U[i++] = u;
    set_observation_vectors(tail...);
}
template <typename T, int N>
template <typename... Tail>
void ECF<T,N>::update_filter(Vec3<T> w, T dt, Tail... tail)
{
    i = 0;
    set_observation_vectors(tail...);
    Vec3<T> mes{0,0,0};
    for(int n = 0; n < N;++n){
        mes += K[n]*cross(U[n], rotate_vec(conjugate(q),V[n]));
        //mes += K[n]*cross(rotate_vec(q,U[n]), V[n]);
    }
    update_attitude(w - b + kp*mes, dt);
    Vec3<T> dot_b = -ki*mes;
    b += dt*dot_b;
}
template <typename T, int N>
void ECF<T,N>::update_attitude(const Vec3<T>& w, const T& dt)
{
    q = integrate_euler(q, w, dt);
}
template <typename T, int N>
Quaternion<T> ECF<T,N>::attitude_kinematics(const Unit_Quaternion<T>& q, const Vec3<T>& w)
{
    Quaternion<T> qw{w};
    return static_cast<T>(1)/2*(q*qw);
}
template <typename T, int N>
Unit_Quaternion<T> ECF<T,N>::integrate_euler(const Unit_Quaternion<T>& q, const Vec3<T>& w, const T& dt)
{
    Quaternion<T> p{q};
    Quaternion<T> dot_q = attitude_kinematics(q, w);
    p += dot_q*dt;
    return {p};
}
#endif // EXPLICIT_COMPLEMENTARY_FILTER_H
