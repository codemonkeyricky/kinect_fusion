// Copyright 2020 Vladimir
// Author: Vladimir
#include "ICP.h"

#include <chrono>
#include <iostream>
#include <memory>
#include <utility>

// #include <xmmintrin.h>
// #include <emmintrin.h>
// #include <smmintrin.h>
#include <immintrin.h>

typedef std::array<float, 4> vector4f;
typedef std::array<float, 16> matrix4f;

matrix4f getRotation(
    const Eigen::Matrix4f &mat)
{
    matrix4f r;
    for (auto i = 0; i < 3; ++i)
        for (auto j = 0; j < 3; ++j)
            r[i * 4 + j] = mat(i, j);
    return r;
}

vector4f getTranslation(
    const Eigen::Matrix4f &mat)
{
    vector4f t;
    t[0] = mat(0, 3);
    t[1] = mat(1, 3);
    t[2] = mat(2, 3);
    t[3] = 1;
    return t;
}

inline vector4f convertToArray(
    const Eigen::Vector3f &v)
{
    vector4f rv;
    rv[0] = v[0];
    rv[1] = v[1];
    rv[2] = v[2];
    rv[3] = 1;
    return rv;
}

// __attribute__((optimize("O0")))
inline matrix4f convertToArray(
    const Eigen::Matrix3f &m)
{
    matrix4f rv = {};
    for (auto i = 0; i < 3; ++i)
        for (auto j = 0; j < 3; ++j)
            rv[i * 4 + j] = m.data()[j * 3 + i];
    return rv;
}

inline vector4f rotate_translate(
    const vector4f &v,
    const matrix4f &r,
    const vector4f &t)
{
    vector4f rv = {};
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
            rv[i] += r[i * 4 + j] * v[j];
        rv[i] += t[i];
    }
    return rv;
}

// __attribute__((optimize("O0")))
inline vector4f rotate_normalize(
    const vector4f &v,
    const matrix4f &r)
{
    vector4f rv = {};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            rv[i] += r[i * 4 + j] * v[j];

    for (int i = 0; i < 4; ++i)
        rv[i] /= rv[2];

    return rv;
}

// __attribute__((optimize("O0")))
inline vector4f get_valid_mask(
    const vector4f &vv)
{
    auto v = _mm_set_ps(vv[3], vv[2], vv[1], vv[0]);
    // INF substract itself is still INF
    __m128 self_sub_v8 = _mm_sub_ps(v, v);
    // Count 
    auto has_inf = _mm_movemask_epi8(_mm_castps_si128(self_sub_v8));
    // Broadcast 
    auto inf_mask = _mm_set1_ps(has_inf);   // either all zeroes or all non-zero
    auto zeros = _mm_set1_ps(0.0f);
    auto valid = _mm_cmp_ps(zeros, inf_mask, _CMP_EQ_OQ); // Greater-than comparison with maxThreshold

    vector4f rv;
    _mm_storeu_ps(&rv[0], valid);
    return rv;
}

// __attribute__((optimize("O0")))
inline vector4f is_coord_in_range(const vector4f &coord)
{
    auto c = _mm_set_ps(0, 0, coord[1], coord[0]);
    auto min_threshold = _mm_set_ps(0, 0, 0, 0);
    auto max_threshold = _mm_set_ps(0, 0, 480, 640); 

    __m128 a = _mm_cmp_ps(min_threshold, c, _CMP_LT_OS); // Less-than comparison with minThreshold
    __m128 b = _mm_cmp_ps(max_threshold, c, _CMP_GT_OS); // Greater-than comparison with maxThreshold

    // Combine the comparison results
    __m128 r = _mm_or_ps(a, b);

    a = _mm_set1_ps(0.0f);
    b = _mm_set1_ps(-0.0f);
    r = _mm_blendv_ps(a, b, r);

    vector4f rv;
    _mm_storeu_ps(&rv[0], r); 
    return rv;
}

// __attribute__((optimize("O0")))
inline vector4f mask_apply(const vector4f &input, const vector4f &mask)
{
    auto a = _mm_set1_ps(0.0f);
    auto b = _mm_set_ps(input[3], input[2], input[1], input[0]);
    auto mask_mm = _mm_set_ps(mask[3], mask[2], mask[1], mask[0]);

    auto r = _mm_blendv_ps(a, b, mask_mm);

    vector4f rv;
    _mm_storeu_ps(&rv[0], r); 
    return rv;
}

//#include <rsvd/Constants.hpp>
//#include <rsvd/ErrorEstimators.hpp>
//#include <rsvd/RandomizedSvd.hpp>

vector4f output[640 * 480];

std::vector<int> p_index; 

// Helper method to find corresponding points between curent frame and
// previous frame Reference Paper:
// https://www.cvl.iis.u-tokyo.ac.jp/~oishi/Papers/Alignment/Blais_Registering_multiview_PAMI1995.pdf
// Input: curent frame, previous frame, estimated pose of previous
// frame Output: indices of corresponding vertices in curent and
// previous frame Simple version: only take euclidean distance between
// points into consideration without normals Advanced version: Euclidean
// distance between points + difference in normal angles
// __attribute__((optimize("O0")))
// __attribute__((optimize("align-functions=64")))
void ICP::findIndicesOfCorrespondingPoints2(
    const Eigen::Matrix4f &estPose)
{
    auto start = std::chrono::high_resolution_clock::now();

    Eigen::Matrix4f estimatedPose = estPose;
    const auto estimatedPoseInv = estimatedPose.inverse();

    const std::vector<vector4f> &prevVertex = prevFrame.getVertexMapGlobal_vector4f();
    const std::vector<vector4f> &prevNormal = prevFrame.getNormalMapGlobal_vector4f();

    auto r = getRotation(estimatedPoseInv);
    auto t = getTranslation(estimatedPoseInv);
    auto ex = curFrame.getExtrinsicMatrix();
    auto ex_r = getRotation(ex);
    auto ex_t = getTranslation(ex);
    auto in = convertToArray(curFrame.getIntrinsicMatrix());

    int cnt = 0;
    for (size_t k = 0; k < prevVertex.size(); k++)
    {
        /* 
         * Subtract all element by itself. INF minus INF is still INF. Create a mask from the MSB, 
         * non-zero mask means at least one element was INF. 
         */
        auto vertex_valid = get_valid_mask(prevVertex[k]);
        auto normal_valid = get_valid_mask(prevNormal[k]);

        // Transform to global coordinate 
        auto curr_camera = rotate_translate(prevVertex[k], r, t);

        // Project to current camera frame
        auto curr_frame = rotate_translate(curr_camera, ex_r, ex_t);

        // Project to image plane
        auto img_coord = rotate_normalize(curr_frame, in);

        // Range check
        auto in_range_valid = is_coord_in_range(img_coord);

        vector4f rv; 
        img_coord = mask_apply(img_coord, vertex_valid);
        img_coord = mask_apply(img_coord, normal_valid);
        img_coord = mask_apply(img_coord, in_range_valid);

        // TODO: still missing cross-referencing with current frame

        output[k] = img_coord; 
    }

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "### ICP2 duration: " << duration.count() << " us" << std::endl;
}

// __attribute__((optimize("O0")))
void ICP::findIndicesOfCorrespondingPoints3(
    const Eigen::Matrix4f &estPose,
    int k)
{
    #if 0
    auto start = std::chrono::high_resolution_clock::now();

    Eigen::Matrix4f estimatedPose = estPose;
    const auto estimatedPoseInv = estimatedPose.inverse();

    const std::vector<vector4f> &prevVertex = prevFrame.getVertexMapGlobal_vector4f();
    const std::vector<vector4f> &prevNormal = prevFrame.getNormalMapGlobal_vector4f();

    auto r = getRotation(estimatedPoseInv);
    auto t = getTranslation(estimatedPoseInv);
    auto ex = curFrame.getExtrinsicMatrix();
    auto ex_r = getRotation(ex);
    auto ex_t = getTranslation(ex);
    auto in = convertToArray(curFrame.getIntrinsicMatrix());

    int cnt = 0;
    // for (size_t k = 0; k < prevVertex.size(); k++)
    {
        /* 
         * Subtract all element by itself. INF minus INF is still INF. Create a mask from the MSB, 
         * non-zero mask means at least one element was INF. 
         */
        auto vertex_valid = get_valid_mask(prevVertex[k]);
        auto normal_valid = get_valid_mask(prevNormal[k]);

        // Transform to global coordinate 
        auto curr_camera = rotate_translate(prevVertex[k], r, t);

        // Project to current camera frame
        auto curr_frame = rotate_translate(curr_camera, ex_r, ex_t);

        // Project to image plane
        auto img_coord = rotate_normalize(curr_frame, in);

        // Range check
        auto in_range_valid = is_coord_in_range(img_coord);

        vector4f rv; 
        img_coord = mask_apply(img_coord, vertex_valid);
        img_coord = mask_apply(img_coord, normal_valid);
        img_coord = mask_apply(img_coord, in_range_valid);

        if (img_coord[0] && img_coord[1])
            ++cnt;

        output[k] = rv; 
    }

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "### ICP3 duration: " << duration.count() << " us" << std::endl;
    #endif
}
