
#include <immintrin.h>

#include "matrix.h"

vector4f operator*(matrix4f &&c1, vector4f &&c2)
{
    vector4f rv = {};
    for (auto i = 0; i < 4; ++i)
        for (auto j = 0; j < 4; ++j)
            rv[i] += c1[i][j] * c2[j];
    return rv;
}

vector4f operator+(vector4f const &c1, vector4f const &c2)
{
    vector4f rv = {};
    for (auto i = 0; i < 4; ++i)
        rv[i] = c1[i] + c2[i];
    return rv;
}

vector4f operator+(vector4f &&c1, vector4f const &&c2)
{
    vector4f rv = {};
    for (auto i = 0; i < 4; ++i)
        rv[i] = c1[i] + c2[i];
    return rv;
}

// float vector4f::dot(const vector4f &op) const
// {
    
// }