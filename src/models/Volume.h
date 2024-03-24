#pragma once

#include <limits>
#include <unordered_map>
#include <vector>

#include "src/helpers/matrix.h"
#include "src/helpers/Eigen.h"
#include "src/models/Frame.h"
// #include "src/helpers/octree.hpp"

typedef unsigned int uint;

using namespace Eigen;

class Voxel
{
private:
    uint8_t weight;
    Vector4uc color;

    inline auto weight_uint8_to_float(uint8_t weight) const -> float
    {
        return (float)weight;
    };

    inline auto weight_float_to_uint8(float w) const -> uint8_t
    {
        return (uint8_t)std::min(255, (int)w); 
    }

public:
    Voxel() {}

    Voxel(float value_, float weight_, Vector4uc color_) : weight{weight_}, color{color_} {}

    inline float getWeight()
    {
        return weight_uint8_to_float(weight);
    }
    
    inline void setWeight(float w)
    {
        weight = weight_float_to_uint8(w);
    }

    Vector4uc getColor()
    {
        return color;
    }

    inline void setColor(Vector4uc c)
    {
        color = c;
    }
};

// Hash function for Eigen matrix and vector.
// The code is from `hash_combine` function of the Boost library. See
// http://www.boost.org/doc/libs/1_55_0/doc/html/hash/reference.html#boost.hash_combine .
template<typename T>
struct matrix_hash : std::unary_function<T, size_t> {
    std::size_t operator()(T const& matrix) const {
        // Note that it is oblivious to the storage order of Eigen matrix (column- or
        // row-major). It will give you the same hash value for two different matrices if they
        // are the transpose of each other in different storage order.
        size_t seed = 0;
        for (size_t i = 0; i < matrix.size(); ++i) {
            auto elem = *(matrix.data() + i);
            seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
        return seed;
    }
};

//! A regular volume dataset
class Volume
{
public:
    Volume();

    //! Initializes an empty volume dataset.
    Volume(Vector3f &min_, Vector3f &max_, float voxel_size = 0.025f, uint dim = 1);
    Volume(int voxels_per_chunk_side = 128, float voxel_size = 0.025f);

    ~Volume();

    inline static Vector3i intCoords(const Vector3f& p) {
        Vector3i coord{ 0, 0, 0 };

        coord[0] = int(p[0]);
        coord[1] = int(p[1]);
        coord[2] = int(p[2]);

        return coord;
    }

    // estimate the normal for a point in voxel grid coordinates using voxel grid by calculating the numerical derivative of TSDF
    Vector3f calculateNormal(const Vector3f& point);

    // trilinear interpolation of a point in voxel grid coordinates to get SDF at the point
    // float trilinearInterpolation(const Vector3f &p) const;
    float trilinearInterpolation(const vector4f &p) const;

    // using given frame calculate TSDF values for all voxels in the grid
    void integrate(Frame &frame);

    // __attribute__((optimize("O0")))
    inline std::array<vector4i, 2> va_remap(const vector4f &va) const
    {
        vector4i pa;
        for (auto i = 0; i < 4; ++i)
            pa[i] = va[i];
    
        vector4i frame, offset;
        for (auto i = 0; i < 4; ++i)
            frame[i] = pa[i] & ~(chunk_len_in_voxels - 1),
            offset[i] = pa[i] & (chunk_len_in_voxels - 1);
        
        return {frame, offset};
    }

    inline auto tsdf_int8_to_float(int8_t tsdf) const -> float
    {
        return ((float)tsdf / 127.0f);
    };

    inline auto tsdf_float_to_int8(float tsdf) const -> int8_t
    {
        int8_t v = roundf(tsdf * 127.0f);
        v = std::min((int8_t)127, v);
        v = std::max((int8_t)-127, v);
        return v;
    };

    // __attribute__((optimize("O0")))
    inline float getTSDF(const vector4f &va) const
    {
        auto [page, offset] = va_remap(va);
        int8_t tsdf = tsdf_dir[page[0]]
                              [page[1]]
                              [page[2]]
                              [offset[0] * chunk_len_in_voxels * chunk_len_in_voxels + offset[1] * chunk_len_in_voxels + offset[2]];
        return tsdf_int8_to_float(tsdf);
    }

    // __attribute__((optimize("O0")))
    inline void setTSDF(const vector4f &va, float v) const
    {
        auto [page, offset] = va_remap(va);
        tsdf_dir[page[0]]
                [page[1]]
                [page[2]]
                [offset[0] * chunk_len_in_voxels * chunk_len_in_voxels + offset[1] * chunk_len_in_voxels + offset[2]] = tsdf_float_to_int8(v);
    }

    inline Voxel &get(const vector4f &va) const
    {
        auto [frame, offset] = va_remap(va);
        return chunk_dir[frame[0]]
                        [frame[1]]
                        [frame[2]]
                        [offset[0] * chunk_len_in_voxels * chunk_len_in_voxels + offset[1] * chunk_len_in_voxels + offset[2]];
    } 

    void setOrigin(const vector4f &va);

    /*
     *  Voxel to world coordinate
     */
    inline vector4f voxelToWorld(const vector4f &p) const
    {
        vector4f world;
        for (auto i = 0; i < 4; ++i)
            world[i] = (p[i] - (float)voxel_offset[i]) * voxel_size;

        return world;
    }

    /* 
     *  World to voxel coordinate
     */
    inline vector4f worldToVoxel(const vector4f &p) const
    {
        /* Convert world coordinate to vox */
        vector4f vox;
        for (auto i = 0; i < 4; ++i)
            vox[i] = p[i] / voxel_size + (float)voxel_offset[i];

        return vox;
    }

    //! Returns the Data.
    Voxel* getData();

    //! Returns number of cells in x-dir.
    inline uint getDimX() const { return dx; }

    //! Returns number of cells in y-dir.
    inline uint getDimY() const { return dy; }

    //! Returns number of cells in z-dir.
    inline uint getDimZ() const { return dz; }

    //! Sets new min and max points
    void setNewBoundingPoints(Vector3f& min_, Vector3f& max_);

    //! Checks if a voxel at coords (x, y, z) was raycasted
    bool voxelVisited(int x, int y, int z) {
        Vector3i pos = Vector3i{ x, y, z };
        if (visitedVoxels.find(pos) != visitedVoxels.end())
            return true;
        else
            return false;
    }

    //! Checks if a voxel at point p in grid coords was raycasted
    bool voxelVisited(Vector3f& p) {
        Vector3i pi = Volume::intCoords(p);
        return voxelVisited(pi[0], pi[1], pi[2]);
    }

    bool voxelVisited(vector4f &p)
    {
        return voxelVisited((int)p[0], (int)p[1], (int)p[2]);
    }

    //! Adds voxel to visited voxels
    void setVisited(vector4f &voxCoords)
    {
        std::vector<Vector3i> starting_points;
        Vector3i p_int = {voxCoords[0], voxCoords[1], voxCoords[2]};

        starting_points.emplace_back(Vector3i{ p_int[0] + 0, p_int[1] + 0, p_int[2] + 0 });
        starting_points.emplace_back(Vector3i{ p_int[0] - 1, p_int[1] + 0, p_int[2] + 0 });
        starting_points.emplace_back(Vector3i{ p_int[0] + 0, p_int[1] - 1, p_int[2] + 0 });
        starting_points.emplace_back(Vector3i{ p_int[0] - 1, p_int[1] - 1, p_int[2] + 0 });
        starting_points.emplace_back(Vector3i{ p_int[0] + 0, p_int[1] + 0, p_int[2] - 1 });
        starting_points.emplace_back(Vector3i{ p_int[0] - 1, p_int[1] + 0, p_int[2] - 1 });
        starting_points.emplace_back(Vector3i{ p_int[0] + 0, p_int[1] - 1, p_int[2] - 1 });
        starting_points.emplace_back(Vector3i{ p_int[0] - 1, p_int[1] - 1, p_int[2] - 1 });

        for (auto p_int : starting_points) {
            visitedVoxels[Vector3i{ p_int[0] + 0, p_int[1] + 0, p_int[2] + 0 }] = true;
            visitedVoxels[Vector3i{ p_int[0] + 1, p_int[1] + 0, p_int[2] + 0 }] = true;
            visitedVoxels[Vector3i{ p_int[0] + 0, p_int[1] + 1, p_int[2] + 0 }] = true;
            visitedVoxels[Vector3i{ p_int[0] + 1, p_int[1] + 1, p_int[2] + 0 }] = true;
            visitedVoxels[Vector3i{ p_int[0] + 0, p_int[1] + 0, p_int[2] + 1 }] = true;
            visitedVoxels[Vector3i{ p_int[0] + 1, p_int[1] + 0, p_int[2] + 1 }] = true;
            visitedVoxels[Vector3i{ p_int[0] + 0, p_int[1] + 1, p_int[2] + 1 }] = true;
            visitedVoxels[Vector3i{ p_int[0] + 1, p_int[1] + 1, p_int[2] + 1 }] = true;
        }



        //std::cout << voxCoords << std::endl;
        //std::cout << visitedVoxels[voxCoords] << std::endl;
        //if (visitedVoxels.empty())
        //    std::cout << "Bok!\n";
    }

    //! Removes vocel from visited voxels
    void removeVisited(Vector3i& voxCoords) {
        visitedVoxels.erase(voxCoords);
    }

    //! Get visited voxels map
    std::unordered_map<Vector3i, bool, matrix_hash<Vector3i>>& getVisitedVoxels() {
        return visitedVoxels;
    }

    //! Updates the color of a voxel
    void updateColor(Vector3i voxelCoords, Vector4uc& color, bool notVisited);

    //! Updates the color of a voxel for a point p in grid coordinates
    void updateColor(Vector3f point, Vector4uc& color, bool notVisited);

    // __attribute__((optimize("O0")))
    bool isPointInVolume(vector4f &voxel)
    {
        vector4f originVox = worldToVoxel(origin);
        int half = chunk_len_in_voxels / 2;

        vector4f mmin, mmax;
        for (auto i = 0; i < 4; ++i)
            mmin[i] = originVox[i] - half,
            mmax[i] = originVox[i] + half;

        for (auto i = 0; i < 3; ++i)
            if (voxel[i] < mmin[i])
                return false;

        for (auto i = 0; i < 3; ++i)
            if (voxel[i] > mmax[i] - 1)
                return false;

        return true;
    }

    //! Checks if the trilinear interpolation possible for a given point (we have to have 8 surrounding points)
    bool isInterpolationPossible(vector4f &voxel)
    {
        vector4f originVox = worldToVoxel(origin);
        int half = chunk_len_in_voxels / 2;

        vector4f mmin, mmax;
        for (auto i = 0; i < 4; ++i)
            mmin[i] = originVox[i] - half,
            mmax[i] = originVox[i] + half;

        for (auto i = 0; i < 3; ++i)
            if (voxel[i] < mmin[i] + 2)
                return false;

        for (auto i = 0; i < 3; ++i)
            if (voxel[i] > mmax[i] - 3)
                return false;

        return true;
    }

private:
    //! Get index of voxel at (x, y, z)
    inline uint getPosFromTuple(int x, int y, int z) const
    {
        return x * dy * dz + y * dz + z;
    }

    //! Lower left and Upper right corner.
    Vector3f min, max;

    //! max-min
    Vector3f diag;

    float ddx, ddy, ddz;
    float dddx, dddy, dddz;

    //! Number of cells in x, y and z-direction.
    uint dx, dy, dz;

    Voxel* vol;

    uint m_dim;

    //map that tracks raycasted voxels
    std::unordered_map<Vector3i, bool, matrix_hash<Vector3i>> visitedVoxels;

    //! Zeros out the memory
    void zeroOutMemory();

    //! Computes spacing in x,y,z-directions.
    void compute_ddx_dddx();


    /*
     *    DYNAMIC_CHUNK feature
     *  Logical coordinates (VA) starts at (0, 0), but to simplify handling of
     *  negative coordinates everything is offset to positive range. 
     */

    vector4f origin;
    int chunk_len_in_voxels;
    float voxel_size; 
    Voxel *chunk_dir[20][20][20] = {};    ///< Each chunk has chunk_len_in_voxels^3 voxels
    int8_t *tsdf_dir[20][20][20] = {};    ///< Each chunk has chunk_len_in_voxels^3 voxels
    vector4i voxel_offset;

    void gridAlloc(const vector4f &va);

};
