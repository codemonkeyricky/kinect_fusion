
#include <benchmark/benchmark.h>

#include "src/helpers/matrix.h"

namespace bm = benchmark;

static void coord_xform(bm::State &state)
{
    matrix4f rotation_inv, ex_r, in, curr_frame;
    vector4f translation_inv, ex_t, v, curr_camera;
    // auto img_coord = in * curr_frame;

    for (auto _ : state)
        bm::DoNotOptimize(curr_camera = rotation_inv * v + translation_inv);

    // curr_frame = ex_r * curr_camera + ex_t;
    // auto img_coord = in * curr_frame;
    // for (int i = 0; i < 4; ++i)
    //     img_coord[i] /= img_coord[2];
}

BENCHMARK(coord_xform);
