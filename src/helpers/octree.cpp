
#include <array>
#include <vector>
#include <algorithm>
#include <iostream>

using namespace std;

vector<vector<vector<float>>> tree;

static void update(array<int, 3> v, int k, array<int, 6> tltr, array<int, 6> lr, float value)
{
    auto tl = tltr[k * 2];
    auto tr = tltr[k * 2 + 1];
    auto l = lr[k * 2];
    auto r = lr[k * 2 + 1];

    if (l > r)
        return;

    if (l == tl && r == tr)
    {
        if (k < 2)
            update(v, k + 1, tltr, lr, value);
        else
            tree[v[0]][v[1]][v[2]] = value;
    }
    else
    {
        int tm = (tl + tr) / 2;

        auto vv = v;
        auto vtltr = tltr;
        auto vlr = lr;

        vv[k] *= 2;
        vtltr[k * 2 + 1] = tm;
        vlr[k * 2 + 1] = min(r, tm);

        update(vv, k, vtltr, vlr, value);

        vv = v;
        vtltr = tltr;
        vlr = lr;

        vv[k] = vv[k] * 2 + 1;
        vtltr[k * 2] = tm + 1;
        vlr[k * 2] = max(l, tm + 1);

        update(vv, k, vtltr, vlr, value);

        vv = v;
        vv[k] = vv[k] * 2;
        auto a = tree[vv[0]][vv[1]][vv[2]];
        vv = v;
        vv[k] = vv[k * 2] + 1;
        auto b = tree[vv[0]][vv[1]][vv[2]];

        tree[v[0]][v[1]][v[2]] = min({a, b});
    }
}

static float query(array<int, 3> v, int k, array<int, 6> tltr, array<int, 6> lr)
{
    auto tl = tltr[k * 2];
    auto tr = tltr[k * 2 + 1];
    auto l = lr[k * 2];
    auto r = lr[k * 2 + 1];

    if (l > r)
        return 1;

    if (l == tl && r == tr)
    {
        if (k < 2)
            return query(v, k + 1, tltr, lr);
        else
            return tree[v[0]][v[1]][v[2]];
    }
    else
    {
        int tm = (tl + tr) / 2;

        auto vv = v;
        auto vtltr = tltr;
        auto vlr = lr;

        vv[k] *= 2;
        vtltr[k * 2 + 1] = tm;
        vlr[k * 2 + 1] = min(r, tm);

        float a = query(vv, k, vtltr, vlr);

        vv = v;
        vtltr = tltr;
        vlr = lr;

        vv[k] = vv[k] * 2 + 1;
        vtltr[k * 2] = tm + 1;
        vlr[k * 2] = max(l, tm + 1);

        float b = query(vv, k, vtltr, vlr);

        return min({a, b});
    }
}

int main()
{
    int dx = 128, dy = 128, dz = 128;
    tree = vector<vector<vector<float>>>(dx * 4, vector<vector<float>>(dy * 4, vector<float>(dz * 4, 1)));

    array<int, 3> v = {1, 1, 1};
    array<int, 6> tltr = {0, 128, 0, 128, 0, 128};
    update(v, 0, tltr, {3, 3, 3, 3, 3, 3}, -1);

    auto vv = query(v, 0, tltr, {3, 3, 3, 4, 3, 4});
    cout << vv << endl;
}

// vector<int> tree;

// void update(int v, int tl, int tr, int l, int r, int value)
// {
//     if (l > r)
//         return;
//     if (l == tl && tr == r)
//         tree[v] = value;
//     else
//     {
//         int tm = (tl + tr) / 2;
//         update(v * 2,
//                tl, tm,
//                l, min(r, tm), value);
//         update(v * 2 + 1,
//                tm + 1, tr,
//                max(l, tm + 1), r, value);
//         tree[v] = max(tree[v * 2], tree[v * 2 + 1]);
//     }
// }

// int query(int v, int tl, int tr, int l, int r)
// {
//     if (l > r)
//         return -1e9;
//     if (l == tl && tr == r)
//         return tree[v];
//     int tm = (tl + tr) / 2;
//     return max(query(v * 2, tl, tm, l, min(r, tm)),
//                query(v * 2 + 1, tm + 1, tr, max(l, tm + 1), r));
// }

// int main()
// {
//     tree = vector<int>(4 * 128);
//     update(1, 0, 128, 3, 3, -1);
// }