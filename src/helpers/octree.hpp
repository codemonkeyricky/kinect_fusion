#pragma once

#include <algorithm>
#include <vector>
#include <iostream>
#include <cassert> 
#include <array> 

using vvvf = std::vector<std::vector<std::vector<float>>>;
using vvf = std::vector<std::vector<float>>;
using vf = std::vector<float>;

class Octree
{
public:
    Octree(int m); 
    void update(int i, int j, int k, int val);
    float mmin(int lx, int rx, int ly, int ry, int lz, int rz);

private:
    int m;
    vvvf tree, a;

    void update_x(int vx, int lx, int rx, int x, int y, int z, float new_val);
    void update_y(int vx, int lx, int rx,
                  int vy, int ly, int ry,
                  int x, int y, int z, float new_val);
    void update_z(int vx, int lx, int rx,
                  int vy, int ly, int ry,
                  int vz, int lz, int rz,
                  int x, int y, int z, float new_val);
    float sum_x(int vx, int tlx, int trx,
                int lx, int rx,
                int ly, int ry,
                int lz, int rz);
    float sum_y(int vx, int vy,
                int tly, int try_, int ly, int ry,
                int lz, int rz);
    float sum_z(int vx, int vy, int vz,
                int tlz, int trz,
                int lz, int rz);
};