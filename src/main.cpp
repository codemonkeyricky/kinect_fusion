// Copyright 2020 Vladimir
// Author: Vladimir
#include <array>
#include <fstream>
#include <iostream>
#include <chrono>

#include "ICP.h"
#include "raycaster.h"
#include "ray.h"
#include "Frame.h"
#include "Volume.h"
#include "VirtualSensor.h"
#include "Eigen.h"
#include "SimpleMesh.h"
#include "MarchingCubes.h"
#include "Renderer.h"

#define DISTANCE_THRESHOLD 0.05
#define EDGE_THRESHOLD 0.02
#define ANGLE_THRESHOLD 1.05
#define MAX_FRAME_NUM 800
#define MIN_POINT -1.6f, -1.6f, -1.6f
#define MAX_POINT 1.6f, 1.6f, 1.6f
#define VOXEL_SIZE 0.0250f
// #define RESOLUTION 128, 128, 128
// #define RESOLUTION 256, 256, 256
// #define RESOLUTION 512, 512, 512
// #define RESOLUTION 1024, 1024, 1024
#define ICP_ITERATIONS 20

extern int run, halt, writeMesh;

int main()
{
    // Make sure this path points to the data folder
    // std::string filenameIn = "data/rgbd_dataset_freiburg1_xyz/";
    // std::string filenameIn = "data/rgbd_dataset_freiburg2_xyz/";
    // std::string filenameIn = "data/rgbd_dataset_freiburg1_desk2/";
    // std::string filenameIn = "data/rgbd_dataset_freiburg1_floor/";
    // std::string filenameIn = "data/rgbd_dataset_freiburg1_rpy/";
    std::string filenameIn = "data/rgbd_dataset_freiburg1_desk/";
    std::string filenameBaseOut = std::string("output/mesh_");
    std::string filenameBaseOutMC = std::string("output/MCmesh_");

    std::vector<vector4f> camPos;

    // load video
    std::cout << "Initialize virtual sensor..." << std::endl;
    VirtualSensor sensor;
    if (!sensor.Init(filenameIn))
    {
        std::cout << "Failed to initialize the sensor!" << std::endl;
        return -1;
    }

    int frameCount = 0;
    Frame curFrame, prevFrame;
    Vector3f min_point = Vector3f{MIN_POINT};
    Vector3f max_point = Vector3f{MAX_POINT};

    // Volume volume = Volume(min_point, max_point, VOXEL_SIZE, 3);
    Volume volume = Volume(256, 0.015f);
    RayCaster rc = RayCaster(volume);
    Matrix4f identity = Matrix4f::Identity(4, 4); // initial estimate
    Matrix4f pose = identity;

    // ensor([ [ 0.8758, 0.3425, -0.3400, 1.3137 ],
    //         [ 0.4820, -0.5867, 0.6508, 0.8486 ],
    //         [ 0.0234, -0.7338, -0.6789, 1.5192 ],
    //         [ 0.0000, 0.0000, 0.0000, 1.0000 ] ])

    // pose(0, 0) = 0.8758f;
    // pose(0, 1) = 0.3425f;
    // pose(0, 2) = -0.3400f;
    // pose(0, 3) = 1.3137f;

    // pose(1, 0) = 0.4820f; 
    // pose(1, 1) = -0.5867f; 
    // pose(1, 2) = 0.6508f; 
    // pose(1, 3) = 0.8486f;

    // pose(2, 0) = 0.0234f; 
    // pose(2, 1) = -0.7338f; 
    // pose(2, 2) = -0.6789f;
    // pose(2, 3) = 1.5192f;

    // pose(3, 3) = 1.0f; 

    Renderer renderer;

    // vector4i bmin = {0, 0, 0, 0};
    // vector4i bmax = {256, 256, 256, 0};

    while (frameCount < MAX_FRAME_NUM && sensor.ProcessNextFrame())
    {
        auto frame_start = std::chrono::high_resolution_clock::now();

        float *depthMap = sensor.GetDepth();
        BYTE *colorMap = sensor.GetColorRGBX();
        Matrix3f depthIntrinsics = sensor.GetDepthIntrinsics();
        Matrix4f depthExtrinsics = sensor.GetDepthExtrinsics();
        // Matrix4f trajectory = sensor.GetTrajectory();
        Matrix4f trajectoryInv = sensor.GetTrajectory().inverse();
        int depthHeight = sensor.GetDepthImageHeight();
        int depthWidth = sensor.GetDepthImageWidth();

        // std::cout << trajectory;

        curFrame =
            Frame(depthMap, colorMap, depthIntrinsics, depthExtrinsics,
                  trajectoryInv, depthWidth, depthHeight);

        // Do the job
        // test run for pose estimation, uncomment to run

        if (frameCount == 0)
        {
            // std::stringstream ss;
            // ss << filenameBaseOut << frameCount << ".off";
            // if (!curFrame.writeMesh(ss.str(), EDGE_THRESHOLD))
            //     return -1;

            // renderer.update(curFrame.getVertexMapGlobal(), (const char *)curFrame.getColorMap());

            vector4f p; // = {0, 0, 0, 0};
            camPos.push_back(p); 
        }
        else
        {
            {
                auto icp_start = std::chrono::high_resolution_clock::now();

                ICP icp(prevFrame, curFrame, DISTANCE_THRESHOLD, ANGLE_THRESHOLD);
                pose = icp.estimatePose(pose, ICP_ITERATIONS);

                auto icp_end = std::chrono::high_resolution_clock::now();
                std::cout << "ICP latency: " << std::chrono::duration_cast<std::chrono::milliseconds>(icp_end - icp_start).count() << " ms" << std::endl;
            }

            vector4f p;
            p[0] = pose(0, 3);
            p[1] = pose(1, 3);
            p[2] = pose(2, 3);
            p[3] = 0;
            camPos.push_back(p);

            curFrame.setExtrinsicMatrix(curFrame.getExtrinsicMatrix() * pose.inverse());

            {
                auto integrate_start = std::chrono::high_resolution_clock::now();

                volume.setOrigin({0, 0, 0, 0});
                volume.integrate(curFrame);

                auto integrate_end = std::chrono::high_resolution_clock::now();
                std::cout << "Integration latency: " << std::chrono::duration_cast<std::chrono::milliseconds>(integrate_end - integrate_start).count() << " ms" << std::endl;
            }

            {
                auto raycast_start = std::chrono::high_resolution_clock::now();
                
                rc.changeFrame(curFrame);
                curFrame = rc.raycast();

                auto raycast_end = std::chrono::high_resolution_clock::now();
                std::cout << "Raycast latency: " << std::chrono::duration_cast<std::chrono::milliseconds>(raycast_end - raycast_start).count() << " ms" << std::endl;
            }

            {
                auto mc_start = std::chrono::high_resolution_clock::now();

                // extract the zero iso-surface using marching cubes
                SimpleMesh mesh;
                std::unordered_map<Vector3i, bool, matrix_hash<Vector3i>> visitedVoxels = volume.getVisitedVoxels();
                for (auto it = visitedVoxels.begin(); it != visitedVoxels.end(); it++)
                {
                    // std::cout << it->first << std::endl;
                    Vector3i voxelCoords = it->first;
                    ProcessVolumeCell(&volume, voxelCoords[0], voxelCoords[1], voxelCoords[2], 0.00f, &mesh);
                }

                auto mc_end = std::chrono::high_resolution_clock::now();
                std::cout << "MarchingCube latency: " << std::chrono::duration_cast<std::chrono::milliseconds>(mc_end - mc_start).count() << " ms" << std::endl;

                do
                {
                    if (halt)
                        run = false;
                    renderer.update(mesh.getTriangles(), mesh.getVertices(), min_point, max_point, volume, camPos);
                } while (!run);

                if (writeMesh)
                {
                    std::stringstream ss;
                    ss << filenameBaseOutMC << frameCount << ".off";
                    mesh.writeMesh(ss.str());
                }
            }

            // if (frameCount >= 1)
            // {
            //     std::stringstream ss;
            //     ss << filenameBaseOut << frameCount << ".off";
            //     if (!curFrame.writeMesh(ss.str(), EDGE_THRESHOLD))
            //         return -1;
            // }
        }

        prevFrame = curFrame;
        frameCount++;

        auto frame_end = std::chrono::high_resolution_clock::now();
        float frame_latency = std::chrono::duration_cast<std::chrono::milliseconds>(frame_end - frame_start).count();
        std::cout << "### frame latency: "
                  << frame_latency << " ms, "
                  << "FPS: " << 1.0f / (frame_latency / 1000)
                  << std::endl;
    }

    return 0;
}
