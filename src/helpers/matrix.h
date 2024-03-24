#pragma once

#include <array>
#include <cmath>

#include <immintrin.h>

struct alignas(16) vector4i 
{
    std::array<int, 4> data = {};
    int operator [](int i) const {return data[i];}
    int & operator [](int i) {return data[i];}
}; 

struct alignas(16) vector4f 
{
    std::array<float, 4> data = {};
    float operator [](int i) const {return data[i];}
    float & operator [](int i) {return data[i];}

    inline float dot(const vector4f &op) const
    {
        float rv = 0;
        for (auto i = 0; i < 4; ++i)
            rv += data[i] * op[i];
        return rv;
    }

    inline vector4f operator*(const float &op) const
    {
        vector4f rv = {};
        for (auto i = 0; i < 4; ++i)
            rv[i] = data[i] * op;
        return rv; 
    }

    inline vector4f operator/(const float &op) const
    {
        vector4f rv = {};
        for (auto i = 0; i < 4; ++i)
            rv[i] = data[i] / op;
        return rv; 
    }

    inline vector4f operator +(const vector4f &op) const
    {
        vector4f rv = {};
        for (auto i = 0; i < 4; ++i)
            rv[i] = data[i] + op[i];
        return rv; 
    }

    inline vector4f operator+=(const vector4f &op)
    {
        for (auto i = 0; i < 4; ++i)
            this->data[i] += op[i];
        return *this;
    }

    inline vector4f operator/=(const float &op)
    {
        for (auto i = 0; i < 4; ++i)
            this->data[i] /= op;
        return *this;
    }

    inline vector4f operator-(const vector4f &op) const
    {
        auto a = _mm_load_ps((const float *)&data);
        auto b = _mm_load_ps((const float *)&op);
        auto c = _mm_sub_ps(a, b);
        vector4f rv;
        _mm_store_ps((float *)&rv, c);
        return rv;
    }

    inline float squaredNorm() const
    {
        float rv = 0;
        for (auto i = 0; i < 4; ++i)
            rv += data[i] * data[i];
        return rv;
    }

    inline float norm() const
    {
        float rv = 0;
        for (auto i = 0; i < 4; ++i)
            rv += data[i] * data[i];
        return std::sqrt(rv); 
    }
}; 

struct alignas(16) matrix4f
{
    std::array<vector4f, 4> data = {};
    vector4f operator [](int i) const {return data[i];}
    vector4f & operator [](int i) {return data[i];}

    matrix4f() 
    {
        data[0][0] = data[1][1] = data[2][2] = data[3][3] = 1;
    }

    inline vector4f operator *(const vector4f &op) const
    {
        vector4f rv = {};
        for (auto i = 0; i < 4; ++i)
            for (auto j = 0; j < 4; ++j)
                rv[i] += data[i][j] * op[j];
        return rv; 
    }
};

inline vector4f operator*(matrix4f &&c1, vector4f &&c2);

inline vector4f operator+(vector4f const &&c1, vector4f const &&c2);
// inline vector4f operator+(vector4f &c1, vector4f const &c2);
// inline vector4f operator+(vector4f const &c1, vector4f &c2);
// inline vector4f operator+(vector4f &c1, vector4f &c2);