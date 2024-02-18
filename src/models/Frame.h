// Copyright 2020 Vladimir
// Author: Vladimir
#pragma once

#ifndef FRAME_H
#define FRAME_H

#include <vector>
#include <memory>

#include "src/helpers/Eigen.h"
#include "src/helpers/VirtualSensor.h"
#include "src/helpers/matrix.h"

class Frame {
    friend class RayCaster;

public:
    Frame();
    Frame(const Frame& other);
    Frame(const float* depthMap, const BYTE* colorMap,
        const Eigen::Matrix3f& depthIntrinsics,
        const Eigen::Matrix4f& depthExtrinsics,
        const Eigen::Matrix4f& trajectoryInv, int depthWidth, int depthHeight);
    Eigen::Vector3f getVertexGlobal(size_t idx) const;
    Eigen::Vector3f getNormalGlobal(size_t idx) const;
    Eigen::Vector3f getVertex(size_t idx) const;
    Eigen::Vector3f getNormal(size_t idx) const;
    int getVertexCount() const;
    std::vector<Eigen::Vector3f> &getVertexMapGlobal();
    std::vector<vector4f> &getVertexMapGlobal_vector4f();
    std::vector<Eigen::Vector3f>& getNormalMapGlobal();
    std::vector<vector4f> &getNormalMapGlobal_vector4f();
    std::vector<Eigen::Vector3f>& getVertexMap();
    std::vector<Eigen::Vector3f>& getNormalMap();
    int getFrameHeight();
    int getFrameWidth();
    void setExtrinsicMatrix(const Eigen::Matrix4f& extrensicMatrix);
    bool containsImgPoint(Eigen::Vector2i);
    Eigen::Vector3f projectPointIntoFrame(const Eigen::Vector3f& point);
    Eigen::Vector2i projectOntoImgPlane(const Eigen::Vector3f& point);
    const Eigen::Matrix4f getExtrinsicMatrix();
    const Eigen::Matrix3f getIntrinsicMatrix();
    const float* getDepthMap();
    const BYTE* getColorMap();
    bool writeMesh(const std::string& filename, float edgeThreshold);

    //+++++++++++++++++++++++++++++++++++++++++++++
    static Eigen::Vector3f transformPoint(
        Eigen::Vector3f& point,
        const Eigen::Matrix4f& transformation);

    static Eigen::Vector2i perspectiveProjection(
        Eigen::Vector3f& point,
        const Eigen::Matrix3f& intrinsic);

    //+++++++++++++++++++++++++++++++++++++++++++++

private:
    void computeVertexMap(const float* depthMap,
        const Eigen::Matrix3f& depthIntrinsics, int depthWidth,
        int depthHeight);
    void computeNormalMap(int depthWidth, int depthHeight);
    std::vector<Eigen::Vector3f> transformPoints(
        const std::vector<Eigen::Vector3f>& points,
        const Eigen::Matrix4f& transformation);

    std::vector<vector4f> transformPoints(
        const std::vector<Eigen::Vector3f> &points,
        const Eigen::Matrix4f &transformation,
        bool dummy);

    std::vector<Eigen::Vector3f> rotatePoints(
        const std::vector<Eigen::Vector3f>& points,
        const Eigen::Matrix3f& rotation);

    std::vector<vector4f> rotatePoints2(
        const std::vector<Eigen::Vector3f> &points,
        const Eigen::Matrix3f &rotation,
        bool dummy);

    int depthWidth;
    int depthHeight;
    const float* depthMap;
    const BYTE* colorMap;
    std::shared_ptr<std::vector<Eigen::Vector3f>> mVertices;
    std::shared_ptr<std::vector<Eigen::Vector3f>> mNormals;
    std::shared_ptr<std::vector<Eigen::Vector3f>> mVerticesGlobal;
    std::shared_ptr<std::vector<Eigen::Vector3f>> mNormalsGlobal;
    std::shared_ptr<std::vector<vector4f>> mVerticesGlobal_vector4f;
    std::shared_ptr<std::vector<vector4f>> mNormalGlobal_vector4f;
    Eigen::Matrix4f extrinsicMatrix;
    Eigen::Matrix3f intrinsicMatrix;
};

#endif //FRAME_H
