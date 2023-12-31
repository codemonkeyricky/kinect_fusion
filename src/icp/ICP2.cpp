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

#include "Frame.h"

matrix4f getRotation(
    const Eigen::Matrix4f &mat)
{
    matrix4f r;
    for (auto i = 0; i < 3; ++i)
        for (auto j = 0; j < 3; ++j)
            r[i][j] = mat(i, j);
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
            rv[i][j] = m.data()[j * 3 + i];
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
            rv[i] += r[i][j] * v[j];
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
            rv[i] += r[i][j] * v[j];

    for (int i = 0; i < 4; ++i)
        rv[i] /= rv[2];

    return rv;
}

// __attribute__((optimize("O0")))
inline void get_valid_mask(
    const vector4f &__restrict vv,
    vector4f &__restrict rv)
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

    _mm_storeu_ps(&rv[0], valid);
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

std::vector<int> pp;


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
    auto time0 = std::chrono::high_resolution_clock::now();

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

    for (size_t k = 0; k < prevVertex.size(); k++)
    {
        /* 
         * Subtract all element by itself. INF minus INF is still INF. Create a mask from the MSB, 
         * non-zero mask means at least one element was INF. 
         */
        // vector4f vertex_valid, normal_valid; 
        // get_valid_mask(prevVertex[k], vertex_valid);
        // get_valid_mask(prevNormal[k], normal_valid);

        // Transform to global coordinate 
        auto curr_camera = rotate_translate(prevVertex[k], r, t);

        // Project to current camera frame
        auto curr_frame = rotate_translate(curr_camera, ex_r, ex_t);

        // Project to image plane
        auto img_coord = rotate_normalize(curr_frame, in);

        // Range check
        // auto in_range_valid = is_coord_in_range(img_coord);

        // img_coord = mask_apply(img_coord, vertex_valid);
        // img_coord = mask_apply(img_coord, normal_valid);
        // img_coord = mask_apply(img_coord, in_range_valid);

        // TODO: still missing cross-referencing with current frame

        output[k] = img_coord; 

        // vector4f vertex_valid, normal_valid; 
        // get_valid_mask(prevVertex[k], vertex_valid);
        // output[k] = vertex_valid;

        // output[k] = prevVertex[k];
    }

    // size_t curIdx =
    //     prevPointImgCoordCurFrame[1] * curFrame.getFrameWidth() +
    //     prevPointImgCoordCurFrame[0];

    // Eigen::Vector3f curFramePointGlobal = rotation * curFrameVertexMapGlobal[curIdx] + translation;
    // Eigen::Vector3f curFrameNormalGlobal = rotation * curFrameNormalMapGlobal[curIdx];

    // if (curFramePointGlobal.allFinite() &&
    //     (curFramePointGlobal - prevVertex).norm() < distanceThreshold && curFrameNormalGlobal.allFinite() && (std::abs(curFrameNormalGlobal.dot(prevNormal)) / curFrameNormalGlobal.norm() / prevNormal.norm() < normalThreshold))
    // {
    //     indicesOfCorrespondingPoints.push_back(std::make_pair(idx, curIdx));
    // }

    vector4f *__restrict output_ptr = &output[0];
    int cnt = 0;

    auto time1 = std::chrono::high_resolution_clock::now();

    {
        const auto rotation = estimatedPose.block(0, 0, 3, 3);
        const auto translation = estimatedPose.block(0, 3, 3, 1);

        const std::vector<Eigen::Vector3f> &curFrameVertexMapGlobal = curFrame.getVertexMapGlobal();
        const std::vector<Eigen::Vector3f> &curFrameNormalMapGlobal = curFrame.getNormalMapGlobal();

        const std::vector<Eigen::Vector3f> &prevFrameVertexMapGlobal = prevFrame.getVertexMapGlobal();
        const std::vector<Eigen::Vector3f> &prevFrameNormalMapGlobal = prevFrame.getNormalMapGlobal();

        pp.clear();

        for (size_t k = 0; k < prevVertex.size(); k++)
        {
            auto v = prevVertex[k];
            auto n = prevNormal[k];
            if (v[0] != MINF && n[0] != MINF)
            {
                // auto [x, y, z, w] = output[k];
                int x = round(output[k][0]);
                int y = round(output[k][1]);
                if (x >= 0 && x < 640 && y >= 0 && y < 480)
                {
                    Eigen::Vector3f prevVertex = prevFrameVertexMapGlobal[k];
                    Eigen::Vector3f prevNormal = prevFrameNormalMapGlobal[k];

                    size_t curIdx = y * curFrame.getFrameWidth() + x;

                    Eigen::Vector3f curFramePointGlobal = rotation * curFrameVertexMapGlobal[curIdx] + translation;
                    Eigen::Vector3f curFrameNormalGlobal = rotation * curFrameNormalMapGlobal[curIdx];

                    if (curFramePointGlobal.allFinite() &&
                        (curFramePointGlobal - prevVertex).norm() < distanceThreshold && curFrameNormalGlobal.allFinite() && (std::abs(curFrameNormalGlobal.dot(prevNormal)) / curFrameNormalGlobal.norm() / prevNormal.norm() < normalThreshold))
                    {
                        pp.push_back(k);
                    }
                }
            }
        }
    }

    auto time2 = std::chrono::high_resolution_clock::now();

    auto duration1 = std::chrono::duration_cast<std::chrono::microseconds>(time1 - time0);
    auto duration2 = std::chrono::duration_cast<std::chrono::microseconds>(time2 - time1);
    std::cout << "### ICP2 duration #1: " << duration1.count() << " us" << std::endl;
    std::cout << "### ICP2 duration #2: " << duration2.count() << " us" << std::endl;
    std::cout << "### ICP2 cnt: " << cnt << std::endl;
}

std::vector<std::pair<size_t, size_t>> ICP::findIndicesOfCorrespondingPoints3(
    const Eigen::Matrix4f &estPose)
{
    Eigen::Matrix4f estimatedPose = estPose;
    std::vector<std::pair<size_t, size_t>> indicesOfCorrespondingPoints;

    const std::vector<Eigen::Vector3f> &prevFrameVertexMapGlobal = prevFrame.getVertexMapGlobal();
    const std::vector<Eigen::Vector3f> &prevFrameNormalMapGlobal = prevFrame.getNormalMapGlobal();

    const std::vector<Eigen::Vector3f> &curFrameVertexMapGlobal = curFrame.getVertexMapGlobal();
    const std::vector<Eigen::Vector3f> &curFrameNormalMapGlobal = curFrame.getNormalMapGlobal();

    const auto rotation = estimatedPose.block(0, 0, 3, 3);
    const auto translation = estimatedPose.block(0, 3, 3, 1);

    const auto estimatedPoseInv = estimatedPose.inverse();

    const auto rotationInv = estimatedPoseInv.block(0, 0, 3, 3);
    const auto translationInv = estimatedPoseInv.block(0, 3, 3, 1);

    auto start = std::chrono::high_resolution_clock::now();

    // GPU implementation: use a separate thread for every run of the for
    // loop
    for (size_t idx = 0; idx < prevFrameVertexMapGlobal.size(); idx++)
    {
        Eigen::Vector3f prevVertex = prevFrameVertexMapGlobal[idx];
        Eigen::Vector3f prevNormal = prevFrameNormalMapGlobal[idx];
        size_t curIdx =idx; 

        Eigen::Vector3f curFramePointGlobal = rotation * curFrameVertexMapGlobal[curIdx] + translation;
        Eigen::Vector3f curFrameNormalGlobal = rotation * curFrameNormalMapGlobal[curIdx];

        if (curFramePointGlobal.allFinite() &&
            (curFramePointGlobal - prevVertex).norm() < distanceThreshold && curFrameNormalGlobal.allFinite() && (std::abs(curFrameNormalGlobal.dot(prevNormal)) / curFrameNormalGlobal.norm() / prevNormal.norm() < normalThreshold))
        {
            indicesOfCorrespondingPoints.push_back(std::make_pair(idx, curIdx));
        }
    }

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "### ICP3 duration: " << duration.count() << std::endl;

    return indicesOfCorrespondingPoints;
}

