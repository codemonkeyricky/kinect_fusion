#include "Volume.h"

auto Volume::integrate_cpu(
    const std::vector<float> &depth,
    const matrix4f &rotation,
    const vector4f &translation,
    const matrix4f &intrinsics,
    std::vector<uint32_t> &tsdf, std::vector<uint32_t> &weight) -> void
{

}