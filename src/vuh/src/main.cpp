#include <vector>
#include <memory>
#include <chrono>
#include <iostream>

#include "vuh/instance.h"
#include "vuh/array.hpp"
#include "vuh/program.hpp"

#include "src/helpers/matrix.h"

auto main()-> int {
   int len = 256; 
   int w = 640, h = 480;
   auto depth = std::vector<float>(w * h, 1.0f);
   auto tsdf = std::vector<float>(len * len * len, 1.0f);
   auto weight = std::vector<float>(len * len * len, 2.0f);

   auto instance = vuh::Instance();
   auto device = instance.devices().at(0);    // just get the first available device

   auto t0 = std::chrono::high_resolution_clock::now();

   auto d_d = vuh::Array<float>(device, depth);   // create device arrays and copy data
   auto d_y = vuh::Array<float>(device, tsdf);   // create device arrays and copy data
   auto d_x = vuh::Array<float>(device, weight);

   auto t1 = std::chrono::high_resolution_clock::now();

   using Specs = vuh::typelist<uint32_t>;     // shader specialization constants interface
   struct Params{ 
      matrix4f rotation;
      vector4f translation; 
      matrix4f intrinsics;
      uint32_t dw; 
      uint32_t dh; 
      uint32_t l;
      };    // shader push-constants interface
   auto program = vuh::Program<Specs, Params>(device, "bazel-bin/src/vuh/saxpy.spv"); // load shader

   matrix4f rot;
   vector4f trans; 
   matrix4f intrs; 
   program.grid(32, 32,32).spec(64)({rot, trans, intrs, w, h, len}, d_d, d_y, d_x); // run once, wait for completion

   auto t2 = std::chrono::high_resolution_clock::now();

   d_y.toHost(begin(tsdf));                      // copy data back to host

   auto t3 = std::chrono::high_resolution_clock::now();

   std::cout << "Upload: " << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() << " ms" << std::endl;
   std::cout << "Compute: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << " ms" << std::endl;
   std::cout << "Download: " << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count() << " ms" << std::endl;

   for(auto i = 0; i < len * len * len; ++i)
      assert(tsdf[i] == i);

   return 0;
}