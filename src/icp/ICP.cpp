// Copyright 2020 Vladimir
// Author: Vladimir

#include <chrono>
#include <iostream>
#include <memory>
#include <utility>

#include "ICP.h"
#include "Frame.h"

//#include <rsvd/Constants.hpp>
//#include <rsvd/ErrorEstimators.hpp>
//#include <rsvd/RandomizedSvd.hpp>

ICP::ICP(Frame &_prevFrame, Frame &_curFrame, const double distanceThreshold,
         const double normalThreshold)
    : prevFrame(_prevFrame),
      curFrame(_curFrame),
      distanceThreshold(distanceThreshold),
      normalThreshold(normalThreshold) {}

extern std::vector<int> pp;

std::vector<std::pair<size_t, size_t>> points_custom;

// __attribute__((optimize("O0")))
Matrix4f ICP::estimatePose(
    Eigen::Matrix4f &estimatedPose,
    int iterationsNum)
{
    points_custom.reserve(640 * 480);

    for (int iteration = 0; iteration < iterationsNum; iteration++)
    {
        // auto points_ref = findIndicesOfCorrespondingPoints(estimatedPose);
        
        // Custom
        points_custom.clear();
        findIndicesOfCorrespondingPoints2(estimatedPose, points_custom);

        // // Reference
        // swap(points_ref, points_custom);
        auto points_ref = points_custom;

        // for (auto i = 0; i < points_ref.size(); ++i)
        //     if (points_custom[i] != points_ref[i])
        //     {
        //         std::cout << "### mismatch: " << points_custom[i].first << ", " << points_ref[i].first << std::endl;
        //         assert(0);
        //     }

        assert(points_ref.size());

#if ICP_DEBUG
        std::cout << "# corresponding points: " << points_ref.size()
                  << std::endl;
        std::cout << "# total number of points: "
                  << curFrame.getVertexMap().size() << std::endl;
#endif

        int nPoints = points_ref.size();
        Eigen::Matrix3f rotationEP = estimatedPose.block(0, 0, 3, 3);
        Eigen::Vector3f translationEP = estimatedPose.block(0, 3, 3, 1);

        MatrixXf A = MatrixXf::Zero(nPoints, 6);
        VectorXf b = VectorXf::Zero(nPoints);

        for (auto i = 0; i < nPoints; i++)
        {
            auto pair = points_ref[i];
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


        auto start = std::chrono::high_resolution_clock::now();

        VectorXf x(6), y(6);
        // x = A.bdcSvd(ComputeThinU | ComputeThinV).solve(b);
        x = (A.transpose() * A).ldlt().solve(A.transpose() * b);
        // static float xx[6], yy[6];
        // for (auto i = 0; i < 6; ++i)
        //     xx[i] = x(i), yy[i] = y(i);

        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
#if ICP_DEBUG
        std::cout << "### pose esitmation : " << duration.count() << " us" << std::endl;
#endif

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
#if ICP_DEBUG
        std::cout << "### estimatedPose : " << estimatedPose(0) << ", " << estimatedPose(1) << ", " << estimatedPose(2) << ", " << std::endl;
#endif
    }
    return estimatedPose;
}
