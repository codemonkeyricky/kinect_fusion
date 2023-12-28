// Copyright 2020 Vladimir
// Author: Vladimir
#include "ICP.h"

#include <chrono>
#include <iostream>
#include <memory>
#include <utility>

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

vector4f convertToArray(
    const Eigen::Vector3f &v)
{
    vector4f rv;
    rv[0] = v[0];
    rv[1] = v[1];
    rv[2] = v[2];
    rv[3] = 1;
    return rv;
}

matrix4f convertToArray(
    const Eigen::Matrix3f &m)
{
    matrix4f rv = {};
    for (auto i = 0; i < 3; ++i)
        for (auto j = 0; j < 3; ++j)
            rv[i * 4 + j] = m(i, j);
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

static vector4f output[640 * 480];

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
    auto start = std::chrono::high_resolution_clock::now();

    Eigen::Matrix4f estimatedPose = estPose;
    const auto estimatedPoseInv = estimatedPose.inverse();

    const std::vector<vector4f> &prevVertex = prevFrame.getVertexMapGlobal_vector4f();

    auto r = getRotation(estimatedPoseInv);
    auto t = getTranslation(estimatedPoseInv);
    auto ex = curFrame.getExtrinsicMatrix();
    auto ex_r = getRotation(ex);
    auto ex_t = getTranslation(ex);

    for (size_t k = 0; k < prevVertex.size(); k++)
    {
        // Transform to global coordinate 
        auto curr_camera = rotate_translate(prevVertex[k], r, t);

        // Project to current camera frame
        auto curr_frame = rotate_translate(curr_camera, ex_r, ex_t);

        // Project to image plane

        output[k] = curr_frame;
    }

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start);
    std::cout << "### ICP2 duration: " << duration.count() << " ns" << std::endl;

    #if 0
    std::vector<std::pair<size_t, size_t>> indicesOfCorrespondingPoints;

    const std::vector<Eigen::Vector3f> &prevFrameVertexMapGlobal = prevFrame.getVertexMapGlobal();
    const std::vector<Eigen::Vector3f> &prevFrameNormalMapGlobal = prevFrame.getNormalMapGlobal();

    const std::vector<Eigen::Vector3f> &curFrameVertexMapGlobal = curFrame.getVertexMapGlobal();
    const std::vector<Eigen::Vector3f> &curFrameNormalMapGlobal = curFrame.getNormalMapGlobal();

    const auto rotation = estimatedPose.block(0, 0, 3, 3);
    const auto translation = estimatedPose.block(0, 3, 3, 1);


    const auto rotationInv = estimatedPoseInv.block(0, 0, 3, 3);
    const auto translationInv = estimatedPoseInv.block(0, 3, 3, 1);

    auto start = std::chrono::high_resolution_clock::now();

    // GPU implementation: use a separate thread for every run of the for
    // loop
    for (size_t idx = 0; idx < prevFrameVertexMapGlobal.size(); idx++)
    {
        Eigen::Vector3f prevVertex = prevFrameVertexMapGlobal[idx];
        Eigen::Vector3f prevNormal = prevFrameNormalMapGlobal[idx];
        // std::cout << "Curent Point (Camera): " << curPoint[0] << " " <<
        // curPoint[1] << " " << curPoint[2] << std::endl;

        if (prevVertex.allFinite() && prevNormal.allFinite())
        {
            Eigen::Vector3f prevPointCurCamera = rotationInv * prevVertex + translationInv;
            // project point from global camera system into camera system of
            // the current frame
            const Eigen::Vector3f prevPointCurFrame = curFrame.projectPointIntoFrame(prevPointCurCamera);
            // project point from camera system of the previous frame onto the
            // image plane of the current frame
            const Eigen::Vector2i prevPointImgCoordCurFrame = curFrame.projectOntoImgPlane(prevPointCurFrame);

            if (curFrame.containsImgPoint(prevPointImgCoordCurFrame))
            {
                size_t curIdx =
                    prevPointImgCoordCurFrame[1] * curFrame.getFrameWidth() +
                    prevPointImgCoordCurFrame[0];

                Eigen::Vector3f curFramePointGlobal = rotation * curFrameVertexMapGlobal[curIdx] + translation;
                Eigen::Vector3f curFrameNormalGlobal = rotation * curFrameNormalMapGlobal[curIdx];

                if (curFramePointGlobal.allFinite() &&
                    (curFramePointGlobal - prevVertex).norm() < distanceThreshold && curFrameNormalGlobal.allFinite() && (std::abs(curFrameNormalGlobal.dot(prevNormal)) / curFrameNormalGlobal.norm() / prevNormal.norm() < normalThreshold))
                {
                    indicesOfCorrespondingPoints.push_back(std::make_pair(idx, curIdx));
                }
            }
        }
    }

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "### ICP duration: " << duration.count() << std::endl;

    return indicesOfCorrespondingPoints;
    #endif
}