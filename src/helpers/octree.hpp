#pragma once

#include <algorithm>
#include <vector>
#include <iostream>
#include <cassert> 
#include <array> 

using vvvf = std::vector<std::vector<std::vector<bool>>>;
using vvf = std::vector<std::vector<bool>>;
using vf = std::vector<bool>;

class Octree
{
public:
    // struct Vox 
    // {
    //     float v; 
    //     float w; 
    //     uint32_t unused; 
    // };

    Octree(int m);
    // void build(Vox *vol, int m);
    void update(int i, int j, int k, bool val);
    bool query_min(int lx, int rx, int ly, int ry, int lz, int rz);

private:
    int m;
    vvvf tree, a;

    void update_x(int vx, int lx, int rx, int x, int y, int z, bool new_val);
    void update_y(int vx, int lx, int rx,
                  int vy, int ly, int ry,
                  int x, int y, int z, bool new_val);
    void update_z(int vx, int lx, int rx,
                  int vy, int ly, int ry,
                  int vz, int lz, int rz,
                  int x, int y, int z, bool new_val);
    bool sum_x(int vx, int tlx, int trx,
                int lx, int rx,
                int ly, int ry,
                int lz, int rz);
    bool sum_y(int vx, int vy,
                int tly, int try_, int ly, int ry,
                int lz, int rz);
    bool sum_z(int vx, int vy, int vz,
                int tlz, int trz,
                int lz, int rz);

#if 0
    void build_x(Vox *vol, int vx, int lx, int rx);
    void build_y(Vox *vol, int vx, int lx, int rx,
                 int vy, int ly, int ry);
    void build_z(Vox *vol, int vx, int lx, int rx,
                 int vy, int ly, int ry,
                 int vz, int lz, int rz);
#endif
};