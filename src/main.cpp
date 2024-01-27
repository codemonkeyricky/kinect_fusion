// Copyright 2020 Vladimir
// Author: Vladimir
#include <array>
#include <fstream>
#include <iostream>

#include "ICP.h"
#include "RayCaster.h"
#include "Ray.h"
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
#define MIN_POINT -3.5f, -2.0f, -1.0f
#define MAX_POINT 0.5f, 2.0f, 2.5f
#define RESOLUTION 128, 128, 128
// #define RESOLUTION 256, 256, 256
// #define RESOLUTION 512, 512, 512
// #define RESOLUTION 1024, 1024, 1024
#define ICP_ITERATIONS 20

extern int run, halt;

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

    Volume volume = Volume(min_point, max_point, RESOLUTION, 3);
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

    while (frameCount < MAX_FRAME_NUM && sensor.ProcessNextFrame())
    {
        float *depthMap = sensor.GetDepth();
        BYTE *colorMap = sensor.GetColorRGBX();
        Matrix3f depthIntrinsics = sensor.GetDepthIntrinsics();
        Matrix4f depthExtrinsics = sensor.GetDepthExtrinsics();
        Matrix4f trajectory = sensor.GetTrajectory();
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
            ICP icp(prevFrame, curFrame, DISTANCE_THRESHOLD, ANGLE_THRESHOLD);
            pose = icp.estimatePose(pose, ICP_ITERATIONS);
            std::cout << pose << std::endl;

            vector4f p;
            p[0] = pose(0, 3);
            p[1] = pose(1, 3);
            p[2] = pose(2, 3);
            p[3] = 0;
            camPos.push_back(p);

            curFrame.setExtrinsicMatrix(curFrame.getExtrinsicMatrix() * pose.inverse());

            volume.integrate(curFrame);

            // if (frameCount >= 1)
            // {
            //     std::stringstream ss;
            //     ss << filenameBaseOut << "raw_" << frameCount << ".off";
            //     if (!curFrame.writeMesh(ss.str(), EDGE_THRESHOLD))
            //         return -1;
            // }

            rc.changeFrame(curFrame);
            curFrame = rc.rayCast();

            // if (frameCount % 2 == 1)
            {
                // TODO

                std::cout << "Marching Cubes started..." << std::endl;
                // extract the zero iso-surface using marching cubes
                SimpleMesh mesh;
                std::unordered_map<Vector3i, bool, matrix_hash<Vector3i>> visitedVoxels = volume.getVisitedVoxels();
                for (auto it = visitedVoxels.begin(); it != visitedVoxels.end(); it++)
                {
                    // std::cout << it->first << std::endl;
                    Vector3i voxelCoords = it->first;
                    ProcessVolumeCell(&volume, voxelCoords[0], voxelCoords[1], voxelCoords[2], 0.00f, &mesh);
                }
                std::cout << "Marching Cubes done! " << mesh.getVertices().size() << " " << mesh.getTriangles().size() << std::endl;


                do
                {
                    if (halt)
                        run = false;
                    renderer.update(mesh.getTriangles(), mesh.getVertices(), min_point, max_point, volume, camPos);
                } while (!run);

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
    }

    return 0;
}
