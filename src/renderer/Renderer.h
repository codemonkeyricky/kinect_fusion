#pragma once

#include <memory>
#include "Eigen.h"

class Renderer
{
public:
    Renderer();
    void update(std::vector<Eigen::Vector3f> &vertices, const char *colorMap);
};