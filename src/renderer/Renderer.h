#pragma once

#include <memory>

#include "SimpleMesh.h"

class Renderer
{
public:
    Renderer();
    void update(std::vector<Eigen::Vector3f> &vertices, const char *colorMap);
    void update(std::vector<Triangle> &triangles, std::vector<Vertex> &vertices);
};