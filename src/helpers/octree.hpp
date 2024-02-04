#pragma once

#include <algorithm>
#include <vector>
#include <iostream>
#include <cassert> 
#include <array> 

using vvvi = std::vector<std::vector<std::vector<int>>>;
using vvi = std::vector<std::vector<int>>;
using vi = std::vector<int>;

class Octree
{
public:
    Octree(int m); 
    void update(int i, int j, int k, int val);
    int sum(int lx, int rx, int ly, int ry, int lz, int rz);

private:
    int m;
    vvvi tree, a;

    void update_x(int vx, int lx, int rx, int x, int y, int z, int new_val);
    void update_y(int vx, int lx, int rx,
                  int vy, int ly, int ry,
                  int x, int y, int z, int new_val);
    void update_z(int vx, int lx, int rx,
                  int vy, int ly, int ry,
                  int vz, int lz, int rz,
                  int x, int y, int z, int new_val);
    int sum_x(int vx, int tlx, int trx,
              int lx, int rx,
              int ly, int ry,
              int lz, int rz);
    int sum_y(int vx, int vy,
              int tly, int try_, int ly, int ry,
              int lz, int rz);
    int sum_z(int vx, int vy, int vz,
              int tlz, int trz,
              int lz, int rz);
};