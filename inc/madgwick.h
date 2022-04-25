#ifndef MADGWICK_H
#define MADGWICK_H
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

template <typename T>
class Madgwick {
private:
    Unit_Quaternion<T> q;
    Vec3<T> b_w;
    T alpha;
    T beta;
    T zeta;
    unsigned int i;
public:
    Madgwick() : q{1,0,0,0}, b_w{0,0,0}, alpha{2}, beta{2}, zeta{2} {}
    Unit_Quaternion<T> get_attitude() {return q;}
    Vec3<T> get_bias() {return b_w;}

    Madgwick(Madgwick<T>& f) = delete;
    Madgwick<T>& operator=(Madgwick<T>& f) = delete;
    Madgwick(Madgwick<T>&& f) = delete;
    Madgwick<T>& operator=(Madgwick<T>&& f) = delete;

    void set_gains(T alpha, T beta, T zeta);
    void reset_filter() {q = {1,0,0,0}; b_w = {0,0,0};}
    void set_reference_vectors(Vec3<T> a, Vec3<T> m);
    void update_filter(Vec3<T> w, T dt, Vec3<T> a, Vec3<T> m);
private:
    void set_Ks(T k);
    void update_attitude(const Vec3<T>& w, const T& dt);
    Quaternion<T> attitude_kinematics(const Unit_Quaternion<T>& q, const Vec3<T>& w);
    Unit_Quaternion<T> integrate_euler(const Unit_Quaternion<T>& q, const Vec3<T>& w, const T& dt);
};
template <typename T>
void Madgwick<T>::set_gains(T alpha, T beta, T zeta)
{
    this->alpha = alpha;
    this->beta = beta;
    this->zeta = zeta;
}
template <typename T>
void Madgwick<T>::update_filter(Vec3<T> w, T dt, Vec3<T> a, Vec3<T> m)
{
    //Gradient descent
    Quaternion<T> q_G{q};

    Vec3<T> a_hat = a/a.magnitude();
    Vec3<T> a_ref = rotate_vec(conjugate(q),Vec3<T>{0,0,1});

    Vec3<T> m_hat = m/m.magnitude();
    Vec3<T> m_i = rotate_vec(q,m_hat);
    Vec3<T> m_ref = rotate_vec(conjugate(q),Vec3<T>{std::sqrt(m_i[0]*m_i[0]+m_i[1]*m_i[1]),0,m_i[2]});

    T f0 = dot({-2*q_G[2], 2*q_G[1],         0}, a_ref - a_hat)
            + dot({-2*m_ref[2]*q_G[2],-2*m_ref[0]*q_G[3]+2*m_ref[2]*q_G[1],2*m_ref[0]*q_G[2]}, m_ref - m_hat);
    T f1 = dot({ 2*q_G[3], 2*q_G[0], -4*q_G[1]}, a_ref - a_hat)
            + dot({2*m_ref[2]*q_G[3],2*m_ref[0]*q_G[2]+2*m_ref[2]*q_G[0],2*m_ref[0]*q_G[3]-4*m_ref[2]*q_G[1]}, m_ref - m_hat);
    T f2 = dot({-2*q_G[0], 2*q_G[3],  4*q_G[2]}, a_ref - a_hat)
            + dot({-m_ref[0]*q_G[2]-2*m_ref[2]*q_G[0],2*m_ref[0]*q_G[1]+2*m_ref[2]*q_G[3],2*m_ref[0]*q_G[0]-4*m_ref[2]*q_G[2]}, m_ref - m_hat);
    T f3 = dot({ 2*q_G[1], 2*q_G[2],         0}, a_ref - a_hat)
            + dot({-4*m_ref[0]*q_G[3]+2*m_ref[2]*q_G[1],-2*m_ref[0]*q_G[0]+2*m_ref[2]*q_G[2],2*m_ref[0]*q_G[1]}, m_ref - m_hat);

    T f_norm = std::sqrt(f0*f0 + f1*f1 + f2*f2 + f3*f3);

    //Prediction from angular velocity
    Vec3<T> w_q = quat_to_vec(static_cast<T>(2)*conjugate(q_G)*Quaternion<T>{f0/f_norm,f1/f_norm,f2/f_norm,f3/f_norm});
    b_w += zeta*w_q*dt;

    auto dot_q = attitude_kinematics(q, w - b_w);
    Quaternion<T> q_w = Quaternion<T>{q} + dt*dot_q;

    T mu = alpha*std::sqrt(dot_q[0]*dot_q[0] + dot_q[1]*dot_q[1] + dot_q[2]*dot_q[2] + dot_q[3]*dot_q[3])*dt;

    T q0 = q_G[0] - mu*f0/f_norm;
    T q1 = q_G[1] - mu*f1/f_norm;
    T q2 = q_G[2] - mu*f2/f_norm;
    T q3 = q_G[3] - mu*f3/f_norm;

    q_G = {q0,q1,q2,q3};

    T y = beta/(mu/dt + beta);

    q = Unit_Quaternion<T>{y*q_G + (1 - y)*q_w};
}
template <typename T>
void Madgwick<T>::update_attitude(const Vec3<T>& w, const T& dt)
{
    q = integrate_euler(q, w, dt);
}
template <typename T>
Quaternion<T> Madgwick<T>::attitude_kinematics(const Unit_Quaternion<T>& q, const Vec3<T>& w)
{
    Quaternion<T> qw{w};
    return static_cast<T>(1)/2*(q*qw);
}
template <typename T>
Unit_Quaternion<T> Madgwick<T>::integrate_euler(const Unit_Quaternion<T>& q, const Vec3<T>& w, const T& dt)
{
    Quaternion<T> p{q};
    Quaternion<T> dot_q = attitude_kinematics(q, w);
    p += dot_q*dt;
    return {p};
}
#endif // MADGWICK_H
