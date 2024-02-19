#include <gtest/gtest.h>

#include "src/models/Volume.h"

int main()
{
    int chunk_len_in_voxels = 256;
    float voxel_size = 0.025f;

    Volume vol(chunk_len_in_voxels, voxel_size);

    Frame frame;

    // Create a 256x256x256 voxel chunk centered at 0,0,0
    vol.setOrigin({0, 0, 0, 0});

    auto vox = vol.worldToVoxel({0, 0, 0, 0});
    EXPECT_EQ(vox[0], 2560);

    vol.integrate(frame);

    vol.setOrigin({chunk_len_in_voxels * voxel_size, chunk_len_in_voxels * voxel_size, chunk_len_in_voxels * voxel_size, 0});

    // for(auto i = -128; i < 128; ++i)
    //     for (auto j = 0; j < 100; ++j)
    //         for (auto k = 0; k < 100; ++k)
    //         {
    //             auto v = vol.get({i, j, k, 0});
    //             v.setTSDF(0);
    //         }

    // for (auto i = 400; i < 500; ++i)
    //     for (auto j = 400; j < 500; ++j)
    //         for (auto k = 400; k < 500; ++k)
    //         {
    //             auto v = vol.get({i, j, k, 0});
    //             v.setTSDF(0);
    //         }

    // for (auto i = -200; i < -100; ++i)
    //     for (auto j = -200; j < -100; ++j)
    //         for (auto k = -200; k < -100; ++k)
    //         {
    //             auto v = vol.get({i, j, k, 0});
    //             v.setTSDF(0);
    //         }


    EXPECT_EQ(0, 1);
}