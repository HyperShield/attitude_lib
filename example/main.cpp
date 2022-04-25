#include <iostream>
#include <cmath>
#include <random>
#include "../inc/quaternion.h"
#include "../inc/vec3.h"
#include "../inc/mat3.h"
#include "attitude.h"
#include "../inc/explicit_complementary_filter.h"
#include "../inc/madgwick.h"

using namespace std;

/*template <typename T>
class Rigid_Body {
private:
    T m;
    Mat3<T> J;
public:
    Rigid_Body(T m, Mat3<T> J) : m{m}, J{J} {}
    void set_mass(T m) {this->m = m;}
    void set_inertia(Mat3<T> J) {this->J = J;}
    T get_mass() {return m;}
    Mat3<T> get_inertia() {return J;}
    virtual void update() = 0;
};*/

void test_func(Unit_Quaternion<float>& q)
{
    cout << q << '\n';
}

int main()
{
    /*Vec3<float> u{1,0,0};
    Vec3<float> v{0,1,0};
    cout << cross(u,v) << '\n';
    Vec3<float> w = Vec3<float>{1,0,0};
    attitude<float> att;
    float dt = 0.01f;
    for(float t = 0; t < 10; t += dt){
        att.update_attitude(w,dt);
    }
    cout << att.get_attitude_quaternion() << '\n';
    cout << att.get_attitude_euler() << '\n';*/

    /*float pi = 3.141592f;
    Unit_Quaternion<float> q_180(0,1,0,0);
    Unit_Quaternion<float> q_90(std::cos(pi/4.0f),0,0,std::sin(pi/4.0f));
    Quaternion<float> v(0,1,2,3);
    Unit_Quaternion<float> q_traf(-q_90[0],q_90[2],-q_90[1],-q_90[3]);

    cout << q_90*q_180*v*conjugate(q_90*q_180) << '\n';
    cout << q_traf*v*conjugate(q_traf) << '\n';
    cout << Unit_Quaternion<float>(1,2,3,4) << '\n';
    cout << Unit_Quaternion<float>(0,0,0,-1)*Unit_Quaternion<float>(1,2,3,4) << '\n';
    cout << Unit_Quaternion<float>{q_180}.conjugate() << '\n';
    cout << expq(v.imag()) << '\n';*/
    using Vec3f = Vec3<float>;
    Vec3f v1{0.0f,0.0f,1.0f};
    Vec3f v2{1.0f,0.0f,0.2f};
    ECF<float,2> F;
    F.set_gains(2.5f,0.2f,0.5f,0.5f);
    F.set_reference_vectors(v1,v2);
    //Unit_Quaternion<float> q{0.5,0.5,0.5,0.5};

    std::mt19937 generator;
    std::normal_distribution<float> w_dist(0.0f,0.1f);
    std::normal_distribution<float> s1_dist(0.0f,0.2f);
    std::normal_distribution<float> s2_dist(0.0f,0.15f);
    Vec3<float> w = Vec3<float>{1,0,0.5};
    attitude<float> att;
    float dt = 0.01f;
    Vec3f b_w{0.1f,0.1f,0.1f};

    Madgwick<float> M;
    M.set_gains(2.0f,1.0f,0.2f);
    for(float t = 0; t < 100; t += dt){
        att.update_attitude(w,dt);
        if(t > 5) {
            Vec3f n_w{w_dist(generator),w_dist(generator),w_dist(generator)};
            Vec3f n_1{s1_dist(generator),s1_dist(generator),s1_dist(generator)};
            Vec3f n_2{s2_dist(generator),s2_dist(generator),s2_dist(generator)};
            auto w_m = w + b_w + n_w;
            auto q = att.get_attitude_quaternion();
            auto u1 = rotate_vec(conjugate(q),v1) + n_1;
            auto u2 = rotate_vec(conjugate(q),v2) + n_2;
            F.update_filter(w_m,dt,u1,u2);
            M.update_filter(w_m, dt, u1, u2);
        }
    }
    cout << att.get_attitude_quaternion() << '\n';
    cout << F.get_attitude() << '\n';
    cout << F.get_bias() << '\n';
    cout << M.get_attitude() << '\n';
    cout << M.get_bias() << '\n';
}
