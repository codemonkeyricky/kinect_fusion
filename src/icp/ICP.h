// Copyright 2020 Vladimir
// Author: Vladimir
#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "Eigen.h"
#include "Frame.h"

class ICP
{
public:
    ICP(Frame &_prevFrame, Frame &_curFrame, const double distanceThreshold,
        const double normalThreshold);

    Matrix4f estimatePose(
        Eigen::Matrix4f &estimatedPose,
        int iterationsNum = 10);

    void findIndicesOfCorrespondingPoints2(
        const Eigen::Matrix4f &estimatedPose,
        std::vector<std::pair<size_t, size_t>> &correspondenceIds);

private:
    Frame &prevFrame;
    Frame &curFrame;
    const double distanceThreshold;
    const double normalThreshold;
};
