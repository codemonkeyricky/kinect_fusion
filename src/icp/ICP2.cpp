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
    t[3] = 0;
    return t;
}

inline vector4f convertToArray(
    const Eigen::Vector3f &v)
{
    vector4f rv;
    rv[0] = v[0];
    rv[1] = v[1];
    rv[2] = v[2];
    rv[3] = 0;
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

inline vector4i round(const vector4f &v)
{
    auto vv = _mm_load_ps((const float*)&v);
    auto rrv = _mm_cvtps_epi32(vv);
    vector4i rv;
    _mm_store_si128( (__m128i*)&rv[0], rrv);
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
void ICP::findIndicesOfCorrespondingPoints2(
    const Eigen::Matrix4f &estPose)
{
    auto time0 = std::chrono::high_resolution_clock::now();

    Eigen::Matrix4f estimatedPose = estPose;

    auto rotation = getRotation(estimatedPose);
    auto translation = getTranslation(estimatedPose);

    const auto estimatedPoseInv = estimatedPose.inverse();

    const auto rotationInv = estimatedPoseInv.block(0, 0, 3, 3);
    const auto translationInv = estimatedPoseInv.block(0, 3, 3, 1);

    matrix4f rotation_inv = {}; 
    for (auto i = 0; i < 3; ++i)
        for (auto j = 0; j < 3; ++j)
            rotation_inv[i][j] = rotationInv(i, j);
    
    vector4f translation_inv = {};
    for (auto i = 0; i < 3; ++i)
        translation_inv[i] = translationInv(i, 0);

    auto ex = curFrame.getExtrinsicMatrix();
    auto ex_r = getRotation(ex);
    auto ex_t = getTranslation(ex);
    auto in = convertToArray(curFrame.getIntrinsicMatrix());

    const std::vector<vector4f> &prevVertex = prevFrame.getVertexMapGlobal_vector4f();
    const std::vector<vector4f> &prevNormal = prevFrame.getNormalMapGlobal_vector4f();

    for (size_t k = 0; k < prevVertex.size(); k++)
    {
        auto curr_camera = rotation_inv * prevVertex[k] + translation_inv;
        auto curr_frame = ex_r * curr_camera + ex_t;
        auto img_coord = in * curr_frame;
        for (int i = 0; i < 4; ++i)
            img_coord[i] /= img_coord[2];
        output[k] = img_coord; 
    }
    
    int cnt = 0;

    auto time1 = std::chrono::high_resolution_clock::now();

    {
        // const auto rotation = estimatedPose.block(0, 0, 3, 3);
        // const auto translation = estimatedPose.block(0, 3, 3, 1);

        const std::vector<vector4f> &curVertex = curFrame.getVertexMapGlobal_vector4f();
        const std::vector<vector4f> &curNormal = curFrame.getNormalMapGlobal_vector4f();

        // pp.clear();

        for (size_t k = 0; k < prevVertex.size(); k++)
        {
            auto pv = prevVertex[k];
            auto pn = prevNormal[k];
            if (pv[0] != MINF && pn[0] != MINF)
            {
                auto coord = round(output[k]);
                auto x = coord[0], y = coord[1];
                if (x >= 0 && x < 640 && y >= 0 && y < 480)
                {
                    size_t kk = y * 640 + x;
                    auto cv = rotation * curVertex[kk] + translation;
                    auto cn = rotation * curNormal[kk];
                    if (curVertex[kk][0] != MINF && curNormal[kk][0] != MINF)
                        // Note: cv + pv * -1.0f is much faster than cv - pv. Not sure why.
                        if ((cv + pv * -1.0f).squaredNorm() < distanceThreshold * distanceThreshold)
                        // if ((cv + npv)[2] < distanceThreshold * distanceThreshold && (cv + npv)[0] < distanceThreshold * distanceThreshold)
                            ++cnt;

                }
            }
        }
    }

    auto time2 = std::chrono::high_resolution_clock::now();

    auto time3 = std::chrono::high_resolution_clock::now();

    auto time4 = std::chrono::high_resolution_clock::now();

    auto duration1 = std::chrono::duration_cast<std::chrono::microseconds>(time1 - time0);
    auto duration2 = std::chrono::duration_cast<std::chrono::microseconds>(time2 - time1);
    auto duration3 = std::chrono::duration_cast<std::chrono::microseconds>(time3 - time2);
    auto duration4 = std::chrono::duration_cast<std::chrono::microseconds>(time4 - time3);
    std::cout << "### ICP2 duration #1: " << duration1.count() << " us" << std::endl;
    std::cout << "### ICP2 duration #2: " << duration2.count() << " us" << std::endl;
    std::cout << "### ICP2 duration #3: " << duration3.count() << " us" << std::endl;
    // std::cout << "### ICP2 duration #4: " << duration4.count() << " us, pixel copied = " << cnt << std::endl;
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

