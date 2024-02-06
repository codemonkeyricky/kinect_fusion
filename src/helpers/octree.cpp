#include "octree.hpp"

using namespace std;

#if 0
void Octree::build_z(Vox *vol, int vx, int lx, int rx,
                     int vy, int ly, int ry,
                     int vz, int lz, int rz)
{
    array<int, 3> x, y, z;
    x = {vx, lx, rx};
    y = {vy, ly, ry};
    z = {vz, lz, rz};

    if (lz == rz)
    {
        if (lx == rx && ly == ry)
            tree[vx][vy][vz] = (char)vol[lx * m * m + ly * m + lz].v;
        else if (lx != rx)
            tree[vx][vy][vz] = min(tree[vx * 2][vy][vz], tree[vx * 2 + 1][vy][vz]);
        else if (ly != ry)
            tree[vx][vy][vz] = min(tree[vx][vy * 2][vz], tree[vx][vy * 2 + 1][vz]);
        else
            tree[vx][vy][vz] = min(tree[vx * 2][vy * 2][vz], tree[vx * 2 + 1][vy * 2 + 1][vz]);
    }
    else
    {
        int mz = (lz + rz) / 2;
        build_z(vol, vx, lx, rx,
                vy, ly, ry,
                vz * 2, lz, mz);
        build_z(vol, vx, lx, rx,
                vy, ly, ry,
                vz * 2 + 1, mz + 1, rz);

        tree[vx][vy][vz] = min(tree[vx][vy][vz * 2], tree[vx][vy][vz * 2 + 1]);
    }
}

void Octree::build_y(Vox *vol, int vx, int lx, int rx,
                     int vy, int ly, int ry)
{
    if (ly != ry)
    {
        int my = (ly + ry) / 2;
        build_y(vol, vx, lx, rx, vy * 2, ly, my);
        build_y(vol, vx, lx, rx, vy * 2 + 1, my + 1, ry);
    }
    build_z(vol, vx, lx, rx,
            vy, ly, ry,
            1, 0, m - 1);
}

void Octree::build_x(Vox *vol, int vx, int lx, int rx)
{
    if (lx != rx)
    {
        int mx = (lx + rx) / 2;
        build_x(vol, vx * 2, lx, mx);
        build_x(vol, vx * 2 + 1, mx + 1, rx);
    }
    build_y(vol, vx, lx, rx, 1, 0, m - 1);
}
#endif

float Octree::sum_z(int vx, int vy, int vz,
                  int tlz, int trz,
                  int lz, int rz)
{
    if (lz > rz)
        return 1e9;

    if (lz == tlz && trz == rz)
        return tree[vx][vy][vz];

    int tmz = (tlz + trz) / 2;

    return min(sum_z(vx, vy, vz * 2,
                     tlz, tmz,
                     lz, min(rz, tmz)),
               sum_z(vx, vy, vz * 2 + 1,
                     tmz + 1, rz,
                     max(lz, tmz + 1), rz));
}

float Octree::sum_y(int vx, int vy,
          int tly, int try_, int ly, int ry,
          int lz, int rz)
{
    if (ly > ry)
        return 1e9;

    if (ly == tly && try_ == ry)
        return sum_z(vx, vy,
                     1, 0, m - 1,
                     lz, rz);

    int tmy = (tly + try_) / 2;
    return min(sum_y(vx, vy * 2,
                     tly, tmy, //  updated v range
                     ly, min(ry, tmy),
                     lz, rz),
               sum_y(vx, vy * 2 + 1,
                     tmy + 1, try_, // updated v range
                     max(ly, tmy + 1), ry,
                     lz, rz));
}

float Octree::sum_x(int vx, int tlx, int trx,
                  int lx, int rx,
                  int ly, int ry,
                  int lz, int rz)
{
    if (lx > rx)
        return 1e9;

    if (lx == tlx && trx == rx)
        return sum_y(vx,
                     1, 0, m - 1, // v1 of y
                     ly, ry,
                     lz, rz);

    int tmx = (tlx + trx) / 2;
    return min(sum_x(vx * 2,
                     tlx, tmx, // updated range
                     lx, min(rx, tmx),
                     ly, ry, lz, rz),
               sum_x(vx * 2 + 1,
                     tmx + 1, trx, // updated range
                     max(lx, tmx + 1), rx,
                     ly, ry, lz, rz));
}

void Octree::update_z(int vx, int lx, int rx,
                      int vy, int ly, int ry,
                      int vz, int lz, int rz,
                      int x, int y, int z, bool new_val)
{
    array<int, 3> xx, yy, zz;
    xx = {vx, lx, rx};
    yy = {vy, ly, ry};
    zz = {vz, lz, rz};

    if (lz == rz)
    {
        if (lx == rx && ly == ry)
            tree[vx][vy][vz] = new_val;
        else if (lx != rx)
            tree[vx][vy][vz] = min(tree[vx * 2][vy][vz], tree[vx * 2 + 1][vy][vz]);
        else if (ly != ry)
            tree[vx][vy][vz] = min(tree[vx][vy * 2][vz], tree[vx][vy * 2 + 1][vz]);
        else
            tree[vx][vy][vz] = min(tree[vx * 2][vy * 2][vz], tree[vx * 2 + 1][vy * 2 + 1][vz]);
    }
    else
    {
        int mz = (lz + rz) / 2;
        if (z <= mz)
            update_z(vx, lx, rx,
                     vy, ly, ry,
                     vz * 2, lz, mz,
                     x, y, z, new_val);
        else
            update_z(vx, lx, rx,
                     vy, ly, ry,
                     vz * 2 + 1, mz + 1, rz,
                     x, y, z, new_val);

        tree[vx][vy][vz] = min(tree[vx][vy][vz * 2], tree[vx][vy][vz * 2 + 1]);
    }
}

void Octree::update_y(int vx, int lx, int rx,
                      int vy, int ly, int ry,
                      int x, int y, int z, bool new_val)
{
    if (ly != ry)
    {
        int my = (ly + ry) / 2;
        if (y <= my)
            update_y(vx, lx, rx,     // range x
                     vy * 2, ly, my, // reduce range y
                     x, y, z, new_val);
        else
            update_y(vx, lx, rx,             // range x
                     vy * 2 + 1, my + 1, ry, // reduce range y
                     x, y, z, new_val);
    }
    update_z(vx, lx, rx,  // range x
             vy, ly, ry,  // range y
             1, 0, m - 1, // range z - all
             x, y, z, new_val);
}

void Octree::update_x(int vx, int lx, int rx, int x, int y, int z, bool new_val)
{
    if (lx != rx)
    {
        int mx = (lx + rx) / 2;
        if (x <= mx)
            update_x(vx * 2,
                     lx, mx,
                     x, y, z, new_val);
        else
            update_x(vx * 2 + 1,
                     mx + 1, rx,
                     x, y, z, new_val);
    }
    update_y(vx,
             lx, rx,
             1, 0, m - 1,
             x, y, z, new_val);
}

// void Octree::build(Vox *volume, int mm)
// {
//     assert(m == mm);
//     build_x(volume, 1, 0, m - 1);
// }

void Octree::update(int i, int j, int k, bool val)
{
    update_x(1, 0, m - 1,
             i, j, k, val);
}

float Octree::query_min(int lx, int rx, int ly, int ry, int lz, int rz)
{
    return sum_x(1, 0, m - 1,
                 lx, rx, ly, ry, lz, rz);
}

Octree::Octree(int m) : m(m)
{
    tree = vvvf(4 * m, vvf(4 * m, vf(4 * m, 1)));
}