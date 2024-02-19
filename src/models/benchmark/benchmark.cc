
#include <benchmark/benchmark.h>

namespace bm = benchmark;

static void i32_addition(bm::State &state) {
    int32_t a, b, c;
    for (auto _ : state)
        c = a + b;
}

BENCHMARK(i32_addition);
