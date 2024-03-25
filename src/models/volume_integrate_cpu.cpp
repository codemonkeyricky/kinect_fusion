#include "Volume.h"

#define TRUNCATION 0.06f

auto Volume::integrate_cpu(
    const float *depthMap,
    const uint32_t *tsdf,
    const uint32_t *weight,
    const matrix4f &ext_rotation,
    const vector4f &ext_translation,
    const matrix4f &intrinsics, 
    const vector4f &translation
    ) -> void
{
    int width = 640; 
    int height = 480;

    for (int i = 0; i < 256; ++i)
    {
        for (int j = 0; j < 256; ++j)
        {
            for (int k = 0; k < 256; ++k)
            {
				vector4f p = {i, j, k, 0};

				// project the grid point into image space
				auto pg = voxelToWorld(p);
				auto pc = ext_rotation * pg + ext_translation;
				auto pi = intrinsics * pc;

				if (pi[0] >= 0 && pi[0] < 640 * pi[2] && pi[1] >= 0 && pi[1] < 480 * pi[2])
				{
					for (int i = 0; i < 4; ++i)
						pi[i] = round(pi[i] / pi[2]);

					// get the depth of the point
					int index = pi[1] * width + pi[0];
                    float depth = depthMap[index];

                    // calculate the sdf value
                    float lambda = (pc / pc[2]).norm();
                    float sdf = depth - ((pg - translation) / lambda).norm();

					// get the previous value and weight
                    float tsdf = getTSDF({i, j, k, 0});

					if (sdf >= -TRUNCATION && depth != MINF)
					{
						float current_tsdf = std::min(1.0f, sdf / TRUNCATION);
						float current_weight = 1.0f;
						float old_tsdf = tsdf; // voxel.getTSDF();
						float old_weight = getWeight({i, j, k, 0});

						auto updated_tsdf = (old_weight * old_tsdf + current_weight * current_tsdf) / (old_weight + current_weight);
						auto updated_weight = old_weight + current_weight;

						setTSDF({i, j, k, 0}, updated_tsdf);
						setWeight({i, j, k, 0}, updated_weight);
					}
				}
			}
        }
    }
}