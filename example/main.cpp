#include <iostream>
#include <cmath>
#include "../inc/quaternion.h"
#include "../inc/vec3.h"
#include "../inc/mat3.h"
#include "attitude.h"

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

    float pi = 3.141592f;
    Unit_Quaternion<float> q_180(0,1,0,0);
    Unit_Quaternion<float> q_90(std::cos(pi/4.0f),0,0,std::sin(pi/4.0f));
    Quaternion<float> v(0,1,2,3);
    Unit_Quaternion<float> q_traf(-q_90[0],q_90[2],-q_90[1],-q_90[3]);

    cout << q_90*q_180*v*conjugate(q_90*q_180) << '\n';
    cout << q_traf*v*conjugate(q_traf) << '\n';
    cout << Unit_Quaternion<float>(1,2,3,4) << '\n';
    cout << Unit_Quaternion<float>(0,0,0,-1)*Unit_Quaternion<float>(1,2,3,4) << '\n';
    cout << Unit_Quaternion<float>{q_180}.conjugate() << '\n';
    cout << expq(v.imag()) << '\n';
}
