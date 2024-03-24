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
   int total = len * len * len/4; 
   auto depth = std::vector<float>(w * h, 1.0f);
   auto tsdf = std::vector<uint32_t>(total, 0x7f7f7f7f);
   auto weight = std::vector<uint32_t>(total, 0x01010101);

   auto instance = vuh::Instance();
   auto device = instance.devices().at(0);    // just get the first available device

   auto t0 = std::chrono::high_resolution_clock::now();

   auto d_d = vuh::Array<float>(device, depth);   // create device arrays and copy data
   auto d_y = vuh::Array<uint32_t>(device, tsdf);   // create device arrays and copy data
   auto d_x = vuh::Array<uint32_t>(device, weight);

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

   matrix4f rot, intrs;
   vector4f trans; 
   program.grid(total /64).spec(64)({rot, trans, intrs, w, h, len}, d_d, d_y, d_x); // run once, wait for completion

   auto t2 = std::chrono::high_resolution_clock::now();

   d_y.toHost(begin(tsdf));                      // copy data back to host

   auto t3 = std::chrono::high_resolution_clock::now();

   std::cout << "Upload: " << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() << " ms" << std::endl;
   std::cout << "Compute: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << " ms" << std::endl;
   std::cout << "Download: " << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count() << " ms" << std::endl;

   uint8_t *tsdf_ptr = (uint8_t*)&tsdf[0];
   for(auto i = 0; i < total*4; ++i)
      assert(tsdf_ptr[i] == i % 256);

   return 0;
}