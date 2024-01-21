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
#define MIN_POINT -1.5f, -1.0f, -1.0f
#define MAX_POINT 1.5f, 1.0f, 2.5f
#define RESOLUTION 128, 128, 128
// #define RESOLUTION 256, 256, 256
// #define RESOLUTION 512, 512, 512
// #define RESOLUTION 1024, 1024, 1024
#define ICP_ITERATIONS 20

bool writeBoundingBox(Vector3f &minpt, Vector3f &maxpt)
{
    std::ofstream outFile("output/bounding.off");
    if (!outFile.is_open())
        return false;

    float nx = minpt[0];
    float ny = minpt[1];
    float nz = minpt[2];
    float px = maxpt[0];
    float py = maxpt[1];
    float pz = maxpt[2];

    // vertex
    outFile << "OFF" << std::endl;

    outFile << "8 2 0" << std::endl;

    outFile << px << " " << py << " " << pz << std::endl;
    outFile << px << " " << ny << " " << pz << std::endl;
    outFile << nx << " " << ny << " " << pz << std::endl;
    outFile << nx << " " << py << " " << pz << std::endl;

    outFile << px << " " << py << " " << nz << std::endl;
    outFile << px << " " << ny << " " << nz << std::endl;
    outFile << nx << " " << ny << " " << nz << std::endl;
    outFile << nx << " " << py << " " << nz << std::endl;

    // face
    outFile << "4    0 3 7 4   255 0 0" << std::endl; // +y
    // outFile << "4    4 5 6 7   255 0 0" << std::endl; // -z
    outFile << "4    3 2 6 7   255 0 0" << std::endl; // -x

    outFile.close();
    return true;
}

int main()
{
    // Make sure this path points to the data folder
    // std::string filenameIn = "data/rgbd_dataset_freiburg1_xyz/";
    // std::string filenameIn = "data/rgbd_dataset_freiburg2_xyz/";
    // std::string filenameIn = "data/rgbd_dataset_freiburg1_desk2/";
    // std::string filenameIn = "data/rgbd_dataset_freiburg1_floor/";
    std::string filenameIn = "data/rgbd_dataset_freiburg1_rpy/";
    // std::string filenameIn = "data/rgbd_dataset_freiburg1_desk/";
    std::string filenameBaseOut = std::string("output/mesh_");
    std::string filenameBaseOutMC = std::string("output/MCmesh_");

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

    writeBoundingBox(min_point, max_point);

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
        }
        else
        {
            // std::cout << prevFrame.getVertex(302992) << std::endl;
            // std::cout << curFrame.getVertex(302992) << std::endl;

            ICP icp(prevFrame, curFrame, DISTANCE_THRESHOLD, ANGLE_THRESHOLD);
            // std::vector<std::pair<size_t, size_t>> correspondenceIds(
            //     {{302990, 302990}});

            //   icp.findIndicesOfCorrespondingPoints2(pose);
            //   icp.findIndicesOfCorrespondingPoints2(pose);

            pose = icp.estimatePose(pose, ICP_ITERATIONS);
            std::cout << pose << std::endl;

            curFrame.setExtrinsicMatrix(curFrame.getExtrinsicMatrix() * pose.inverse());

            // {
            //   std::stringstream ss;
            //   ss << filenameBaseOut << frameCount << "_b4.off";
            //   if (!curFrame.writeMesh(ss.str(), EDGE_THRESHOLD))
            //   {
            //     std::cout << "Failed to write mesh!\nCheck file path!" << std::endl;
            //     return -1;
            //   }
            // }

            volume.integrate(curFrame);

            rc.changeFrame(curFrame);
            curFrame = rc.rayCast();

            // renderer.update(curFrame.getVertexMapGlobal(), (const char *)curFrame.getColorMap());

            if (0)
            {
                std::stringstream ss;
                ss << filenameBaseOut << frameCount << ".off";
                if (!curFrame.writeMesh(ss.str(), EDGE_THRESHOLD))
                {
                    std::cout << "Failed to write mesh!\nCheck file path!" << std::endl;
                    return -1;
                }
            }

            // if (frameCount % 2 == 1)
            {
                // TODO
                std::stringstream ss;
                ss << filenameBaseOutMC << frameCount << ".off";

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
                /*
                for (unsigned int x = 0; x < volume.getDimX() - 1; x++)
                {
                    //std::cerr << "Marching Cubes on slice " << x << " of " << volume.getDimX() << std::endl;

                    for (unsigned int y = 0; y < volume.getDimY() - 1; y++)
                    {
                        for (unsigned int z = 0; z < volume.getDimZ() - 1; z++)
                        {
                            ProcessVolumeCell(&volume, x, y, z, 0.00f, &mesh);
                        }
                    }
                }
                */
                std::cout << "Marching Cubes done! " << mesh.getVertices().size() << " " << mesh.getTriangles().size() << std::endl;

                renderer.update(mesh.getTriangles(), mesh.getVertices(), min_point, max_point);

                // write mesh to file
                if (!mesh.writeMesh(ss.str()))
                {
                    std::cout << "ERROR: unable to write output file!" << std::endl;
                    return -1;
                }
            }
        }

        prevFrame = curFrame;
        frameCount++;
    }

    return 0;
}
