#include <algorithm>
#include <vector>
#include <iostream>
#include <cassert> 
#include <array> 

using namespace std;

vector<vector<vector<int>>> t, a;
int m;

void build_z(int vx, int lx, int rx,
             int vy, int ly, int ry,
             int vz, int lz, int rz)
{
    array<int,3> x, y, z;
    x = {vx, lx, rx};
    y = {vy, ly, ry};
    z = {vz, lz, rz};

    if (lz == rz)
    {
        if (lx == rx && ly == ry)
            t[vx][vy][vz] = a[lx][ly][lz];
        else if (lx != rx)
            t[vx][vy][vz] = t[vx * 2][vy][vz] + t[vx * 2 + 1][vy][vz];
        else if (ly != ry)
            t[vx][vy][vz] = t[vx][vy * 2][vz] + t[vx][vy * 2 + 1][vz];
        else
            t[vx][vy][vz] = t[vx * 2][vy * 2][vz] + t[vx * 2 + 1][vy * 2 + 1][vz];
    }
    else
    {
        int mz = (lz + rz) / 2;
        build_z(vx, lx, rx, 
                vy, ly, ry, 
                vz * 2, lz, mz);
        build_z(vx, lx, rx,
                vy, ly, ry, 
                vz * 2 + 1, mz + 1, rz);

        t[vx][vy][vz] = t[vx][vy][vz * 2] + t[vx][vy][vz * 2 + 1];
    }
}

void build_y(int vx, int lx, int rx,
             int vy, int ly, int ry)
{
    if (ly != ry)
    {
        int my = (ly + ry) / 2;
        build_y(vx, lx, rx, vy * 2, ly, my);
        build_y(vx, lx, rx, vy * 2 + 1, my + 1, ry);
    }
    build_z(vx, lx, rx,
            vy, ly, ry,
            1, 0, m - 1);
}

void build_x(int vx, int lx, int rx)
{
    if (lx != rx)
    {
        int mx = (lx + rx) / 2;
        build_x(vx * 2, lx, mx);
        build_x(vx * 2 + 1, mx + 1, rx);
    }
    build_y(vx, lx, rx, 1, 0, m - 1);
}

int sum_z(int vx, int vy, int vz,
          int tlz, int trz,
          int lz, int rz)
{
    if (lz > rz)
        return 0;

    if (lz == tlz && trz == rz)
        return t[vx][vy][vz];

    int tmz = (tlz + trz) / 2;

    return sum_z(vx, vy, vz * 2,
                 tlz, tmz,
                 lz, min(rz, tmz)) +
           sum_z(vx, vy, vz * 2 + 1,
                 tmz + 1, rz,
                 max(lz, tmz + 1), rz);
}

int sum_y(int vx, int vy,
          int tly, int try_, int ly, int ry,
          int lz, int rz)
{
    if (ly > ry)
        return 0;

    if (ly == tly && try_ == ry)
        return sum_z(vx, vy,
                     1, 0, m - 1,
                     lz, rz);

    int tmy = (tly + try_) / 2;
    return sum_y(vx, vy * 2,
                 tly, tmy, //  updated v range
                 ly, min(ry, tmy),
                 lz, rz) +
           sum_y(vx, vy * 2 + 1,
                 tmy + 1, try_, // updated v range
                 max(ly, tmy + 1), ry,
                 lz, rz);
}

int sum_x(int vx, int tlx, int trx,
          int lx, int rx,
          int ly, int ry,
          int lz, int rz)
{
    if (lx > rx)
        return 0;

    if (lx == tlx && trx == rx)
        return sum_y(vx,
                     2, 0, m - 1, // v1 of y
                     ly, ry,
                     lz, rz);

    int tmx = (tlx + trx) / 2;
    return sum_x(vx * 2,
                 tlx, tmx, // updated range
                 lx, min(rx, tmx),
                 ly, ry, lz, rz) +
           sum_x(vx * 2 + 1,
                 tmx + 1, trx, // updated range
                 max(lx, tmx + 1), rx,
                 ly, ry, lz, rz);
}

#if 0
void update_y(int vx, int lx, int rx, int vy, int ly, int ry, int x, int y, int new_val)
{
    if (ly == ry)
    {
        if (lx == rx)
            t[vx][vy] = new_val;
        else
            t[vx][vy] = t[vx * 2][vy] + t[vx * 2 + 1][vy];
    }
    else
    {
        int my = (ly + ry) / 2;
        if (y <= my)
            update_y(vx, lx, rx, vy * 2, ly, my, x, y, new_val);
        else
            update_y(vx, lx, rx, vy * 2 + 1, my + 1, ry, x, y, new_val);
        t[vx][vy] = t[vx][vy * 2] + t[vx][vy * 2 + 1];
    }
}

void update_x(int vx, int lx, int rx, int x, int y, int new_val)
{
    if (lx != rx)
    {
        int mx = (lx + rx) / 2;
        if (x <= mx)
            update_x(vx * 2, lx, mx, x, y, new_val);
        else
            update_x(vx * 2 + 1, mx + 1, rx, x, y, new_val);
    }
    update_y(vx, lx, rx, 1, 0, m - 1, x, y, new_val);
}
#endif

int main()
{
    m = 8;
    a = vector<vector<vector<int>>>(m, vector<vector<int>>(m, vector<int>(m)));
    t = vector<vector<vector<int>>>(4 * m, vector<vector<int>>(4 * m, vector<int>(4 * m)));

    a[0][0][0] = 1;
    a[1][1][0] = 1;
    build_x(1, 0, m - 1);

    assert(sum_x(1, 0, m - 1,
                 2, 3, 2, 3, 0, 3) == 0);
    assert(sum_x(1, 0, m - 1,
                 0, 3, 0, 3, 0, 3) == 2);
}