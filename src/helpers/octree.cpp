
#include <array>
#include <vector>
#include <algorithm>
#include <iostream>

using namespace std;

vector<vector<vector<float>>> tree;

static void update(array<int, 3> v, array<int, 6> tltr, array<int, 6> lr, float value)
{
    auto [xtl, xtr, ytl, ytr, ztl, ztr] = tltr;
    auto [xl, xr, yl, yr, zl, zr] = lr;

    if (xl > xr || yl > yr || zl > zr)
        return;

    if (tltr == lr)
        tree[v[0]][v[1]][v[2]] = value;
    else
    {
        int xtm = (xtl + xtr) / 2;
        int ytm = (ytl + ytr) / 2;
        int ztm = (ztl + ztr) / 2;

        update({v[0] * 2, v[1], v[2]},
               {xtl, xtm,         ytl, ytr, ztl, ztr},
               {xl, min(xr, xtm), yl, yr, zl, zr},
               value);

        update({v[0] * 2 + 1, v[1], v[2]},
               {xtm + 1, xtr,       ytl, ytr, ztl, ztr},
               {max(xl, xtm + 1),  xr, yl, yr, zl, zr},
               value);

        update({v[0], v[1] * 2, v[2]},
               {xtl, xtr, ytl, ytm, ztl, ztr},
               {xl, xr, yl, min(yr, ytm), zl, zr},
               value);

        update({v[0], v[1] * 2 + 1, v[2]},
               {xtl, xtr, ytm + 1, ytr, ztl, ztr},
               {xl, xr, max(yl, ytm + 1), yr, zl, zr},
               value);

        update({v[0], v[1], v[2] * 2},
               {xtl, xtr, ytl, ytr, ztl, ztm},
               {xl, xr, yl, yr, zl, min(zr, ztm)},
               value);

        update({v[0], v[1] * 2 + 1, v[2]},
               {xtl, xtr, ytl, ytr, ztm + 1, ztr},
               {xl, xr, yl, yr, max(zl, ztm + 1), zr},
               value);
    }
}

static float query(array<int, 3> v, array<int, 6> v_range, array<int, 6> r_range)
{
    auto [txl, txr, tyl, tyr, tzl, tzr] = v_range;
	auto [rxl, rxr, ryl, ryr, rzl, rzr] = r_range;

	if (rxl > rxr || ryl > ryr || rzl > rzr)
		return 1;

	if (v_range == r_range)
		return tree[v[0]][v[1]][v[2]];

    int txm = (txl + txr) / 2;
    int tym = (tyl + tyr) / 2;
    int tzm = (tzl + tzr) / 2;

    float a = query({v[0] * 2, v[1], v[2]},
                    {txl, txm,              tyl, tyr, tzl, tzr},
                    {rxl, min(rxr, txm),    ryl, ryr, rzl, rzr});

    float b = query({v[0] * 2 + 1, v[1], v[2]},
                    {txm + 1, txr,              tyl, tyr, tzl, tzr},
                    {max(rxl, txm + 1), rxr,    ryl, ryr, rzl, rzr});

    float c = query({v[0], v[1] * 2, v[2]},
                    {txl, txr, tyl, tym,            tzl, tzr},
                    {rxl, rxr, ryl, min(ryr, tym),  rzl, rzr});

    float d = query({v[0], v[1] * 2 + 1, v[2]},
                    {txl, txr, tym + 1, tyr,            tzl, tzr},
                    {rxl, rxr, max(ryl, tym + 1), ryr,  rzl, rzr});

    float e = query({v[0], v[1], v[2] * 2},
                    {txl, txr, tyl, tyr, tzl, tzm},
                    {rxl, rxr, ryl, ryr, rzl, min(rzr, tzm)});

    float f = query({v[0], v[1] * 2 + 1, v[2]},
                    {txl, txr, tyl, tyr, tzm + 1, tzr},
                    {rxl, rxr, ryl, ryr, max(rzl, tzm + 1), rzr});

    return min({a, b, c, d, e, f});
}

int main()
{
    int dx = 128, dy = 128, dz = 128;
    tree = vector<vector<vector<float>>>(dx * 4, vector<vector<float>>(dy * 4, vector<float>(dz * 4, 1)));

    array<int, 3> v = {1, 1, 1};
    array<int, 6> tltr = {0, 128, 0, 128, 0, 128};
    update(v, tltr, {3, 3, 3, 3, 3, 3}, -1);

    auto vv = query(v, tltr, {3, 3, 3, 3, 3});
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