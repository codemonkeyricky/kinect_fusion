// Copyright 2020 Vladimir
// Author: Vladimir
#include "ICP.h"

#include <chrono>
#include <iostream>
#include <memory>
#include <utility>

ICP::ICP(Frame &_prevFrame, Frame &_curFrame, const double distanceThreshold,
         const double normalThreshold)
    : prevFrame(_prevFrame),
      curFrame(_curFrame),
      distanceThreshold(distanceThreshold),
      normalThreshold(normalThreshold) {}

Matrix4f ICP::estimatePose(
    Eigen::Matrix4f& estimatedPose,
    int iterationsNum
) {

    findIndicesOfCorrespondingPoints2(estimatedPose);

    for (size_t iteration = 0; iteration < iterationsNum; iteration++)
    {
        const std::vector<std::pair<size_t, size_t>> correspondenceIds = findIndicesOfCorrespondingPoints(estimatedPose);

        std::cout << "# corresponding points: " << correspondenceIds.size()
                  << std::endl;
        std::cout << "# total number of points: "
                  << curFrame.getVertexMap().size() << std::endl;

        int nPoints = correspondenceIds.size();
        Eigen::Matrix3f rotationEP = estimatedPose.block(0, 0, 3, 3);
        Eigen::Vector3f translationEP = estimatedPose.block(0, 3, 3, 1);

        MatrixXf A = MatrixXf::Zero(nPoints, 6);
        VectorXf b = VectorXf::Zero(nPoints);

        for (size_t i = 0; i < nPoints; i++)
        {
            auto pair = correspondenceIds[i];
            const auto &x = rotationEP * curFrame.getVertexGlobal(pair.second) + translationEP;
            const auto &y = prevFrame.getVertexGlobal(pair.first);
            const auto &n = prevFrame.getNormalGlobal(pair.first);

            A(i, 0) = n(2) * x(1) - n(1) * x(2);
            A(i, 1) = n(0) * x(2) - n(2) * x(0);
            A(i, 2) = n(1) * x(0) - n(0) * x(1);
            A(i, 3) = n(0);
            A(i, 4) = n(1);
            A(i, 5) = n(2);
            b(i) = n(0) * y(0) + n(1) * y(1) + n(2) * y(2) - n(0) * x(0) -
                   n(1) * x(1) - n(2) * x(2);
        }

        VectorXf x(6);
        x = A.bdcSvd(ComputeThinU | ComputeThinV).solve(b);

        const float alpha = x(0), beta = x(1), gamma = x(2);
        Matrix3f rotation =
            AngleAxisf(alpha, Vector3f::UnitX()).toRotationMatrix() *
            AngleAxisf(beta, Vector3f::UnitY()).toRotationMatrix() *
            AngleAxisf(gamma, Vector3f::UnitZ()).toRotationMatrix();
        Vector3f translation = x.tail(3);

        Matrix4f curentPose = Matrix4f::Identity();
        curentPose.block<3, 3>(0, 0) = rotation;
        curentPose.block<3, 1>(0, 3) = translation;
        estimatedPose = curentPose * estimatedPose;
  }
  return estimatedPose;
}

// void transform_scalar(vector4f *__restrict rv,
//                  const matrix4f *__restrict rotation,
//                  const vector4f *__restrict translation,
//                  const vector4f *__restrict pixel,
//                  const int len)
// {
//   for (int k = 0; k < len; ++k)
//     for (int i = 0; i < 4; ++i)
//     {
//       for (int j = 0; j < 4; ++j)
//         rv[k][i] *= (*rotation)[i * 4 + j] * pixel[k][j];
//       rv[k][i] += (*translation)[i];
//     }
// }

typedef std::array<float, 4> vector4f;
typedef std::array<float, 16> matrix4f;

static std::array<float, 16> getRotation(
    const Eigen::Matrix4f &mat)
{
    std::array<float, 16> r;
    for (auto i = 0; i < 3; ++i)
        for (auto j = 0; j < 3; ++j)
            r[i * 4 + j] = mat(i, j);
    return r;
}

static std::array<float, 4> getTranslation(
    const Eigen::Matrix4f &mat)
{
    std::array<float, 4> t;
    t[0] = mat(0, 3);
    t[1] = mat(1, 3);
    t[2] = mat(2, 3);
    t[3] = 1;
    return t;
}

static std::array<float, 4> convertToArray(
    const Eigen::Vector3f &v)
{
    std::array<float, 4> rv;
    rv[0] = v[0];
    rv[1] = v[1];
    rv[2] = v[2];
    rv[3] = 1;
    return rv;
}

static std::array<float, 16> convertToArray(
    const Eigen::Matrix3f &m)
{
    std::array<float, 16> rv = {};
    for (auto i = 0; i < 3; ++i)
        for (auto j = 0; j < 3; ++j)
            rv[i * 4 + j] = m(i, j);
    return rv;
}

static std::array<float, 4> rotate_translate(
    std::array<float, 4> &v,
    std::array<float, 16> &r,
    std::array<float, 4> &t)
{
    std::array<float, 4> rv = {};
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
            rv[i] += r[i * 4 + j] * v[j];
        rv[i] += t[i];
    }
    return rv;
}

// Helper method to find corresponding points between curent frame and
// previous frame Reference Paper:
// https://www.cvl.iis.u-tokyo.ac.jp/~oishi/Papers/Alignment/Blais_Registering_multiview_PAMI1995.pdf
// Input: curent frame, previous frame, estimated pose of previous
// frame Output: indices of corresponding vertices in curent and
// previous frame Simple version: only take euclidean distance between
// points into consideration without normals Advanced version: Euclidean
// distance between points + difference in normal angles
__attribute__((optimize("O0"))) std::vector<std::pair<size_t, size_t>> ICP::findIndicesOfCorrespondingPoints(
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
        // std::cout << "Curent Point (Camera): " << curPoint[0] << " " <<
        // curPoint[1] << " " << curPoint[2] << std::endl;

#if 1
        auto v = convertToArray(prevVertex);
        auto r = getRotation(estimatedPoseInv);
        auto t = getTranslation(estimatedPoseInv);
        auto curr_camera = rotate_translate(v, r, t);

        auto extrinsics = curFrame.getExtrinsicMatrix();
        auto ex_r = getRotation(extrinsics);
        auto ex_t = getTranslation(extrinsics);
        auto curr_frame = rotate_translate(curr_camera, ex_r, ex_t);

        auto intrinsics = convertToArray(curFrame.getIntrinsicMatrix());

        // asm("#bullshit");
        // volatile auto rv2 = r * v + t;
        // asm("#bullshit");

        // https://stackoverflow.com/questions/30674291/how-to-check-inf-for-avx-intrinsic-m256
        // __m256 self_sub_v8 = _mm128_sub_ps(float_v8, float_v8);
        // return _mm256_movemask_epi8(_mm256_castps_si256(self_sub_v8));

#endif

        if (prevVertex.allFinite() && prevNormal.allFinite())
        {

#if 1
#endif

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
}