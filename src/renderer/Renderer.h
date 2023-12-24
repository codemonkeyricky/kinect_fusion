#pragma once

#include <memory>

#include "SimpleMesh.h"
#include "Volume.h"

class Renderer
{
public:
    Renderer();
    void update(std::vector<Eigen::Vector3f> &vertices, const char *colorMap);
    void update(std::vector<Triangle> &triangles, std::vector<Vertex> &vertices, Vector3f &minpt, Vector3f &maxpt,
                Volume &volume, std::vector<vector4f> &camPos);
};