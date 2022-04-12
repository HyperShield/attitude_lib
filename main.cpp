#include <iostream>
#include <cmath>
#include "quaternion.h"
#include "vec3.h"
#include "mat3.h"
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
    Mat3<float> A = {{1,0,0},{0,1,0},{0,0,1}};
    Mat3<float> B(A);

    Quaternion<float> q{2,0,0,0};
    Unit_Quaternion<float> v{0.5,0.5,0.5,0.5};
    Unit_Quaternion<float> qq{1,0,0,0};
    auto p = v*q;

    cout << p << '\n';
    //cout << 5.0f*v << '\n';
    Vec3<float> w = Vec3<float>{1,0,0};
    attitude<float> att;
    float dt = 0.01f;
    for(float t = 0; t < 10; t += dt){
        att.update_attitude(w,dt);
    }
    cout << att.get_attitude_quaternion() << '\n';
    cout << att.get_attitude_euler() << '\n';
    return 0;
}
