#ifndef MAT_H
#define MAT_H
#include <cassert>
#include <initializer_list>
#include "vec3.h"
template <typename T>
struct row_vec{
    T row[3];
    T operator[](unsigned int i) const {assert(i < 3); return row[i];}
};
template <typename T>
class Mat3 {
private:
    T A[9];
public:
    Mat3();// = default;
    Mat3(Mat3&&) = default;
    Mat3& operator=(Mat3&&) = default;
    Mat3(const Mat3&) = default;
    Mat3& operator=(const Mat3&) = default;
    ~Mat3() = default;
    Mat3(Mat3& M);
    Mat3(std::initializer_list<row_vec<T>> l);
    Mat3(std::initializer_list<T> l) = delete;

    T&      operator()(unsigned int i, unsigned int j)      {assert(i < 3 && j < 3); return A[i*3+j];}
};
template <typename T>
Mat3<T>::Mat3()
{
    for(int i = 0; i < 3; ++i) {
        for(int j = 0; j < 3; ++j) {
            A[i*3 + j] = 0;
        }
    }
}
template <typename T>
Mat3<T>::Mat3(Mat3<T>& M)
{
    for(int i = 0; i < 3; ++i) {
        for(int j = 0; j < 3; ++j) {
            A[i*3 + j] = M(i,j);
        }
    }
}
template <typename T>
Mat3<T>::Mat3(std::initializer_list<row_vec<T>> l)
{
    for(int i = 0; i < 3; ++i) {
        for(int j = 0; j < 3; ++j) {
            (*this)(i,j) = l.begin()[i][j];
        }
    }
}
#endif
