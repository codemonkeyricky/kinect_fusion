
#include <chrono> 

#include "Volume.h"

#define TRUNCATION 0.06f

Volume::Volume() {}

//! Initializes an empty volume dataset.
Volume::Volume(Vector3f &min_, Vector3f &max_, float voxel_size, uint dim)
{
	min = min_;
	max = max_;
	diag = max - min;

	dx = (max[0] - min[0]) / voxel_size;
	dy = (max[1] - min[1]) / voxel_size;
	dz = (max[2] - min[2]) / voxel_size;

	m_dim = dim;
	vol = NULL;

	assert(dx == dy);
	assert(dy == dz);

	vol = new Voxel[dx * dy * dz];
	tree = new Octree(128);

	zeroOutMemory();
	compute_ddx_dddx();
}

Volume::~Volume()
{
	delete[] vol;
};

//! Computes spacing in x,y,z-directions.
void Volume::compute_ddx_dddx()
{
	ddx = 1.0f / (dx - 1);
	ddy = 1.0f / (dy - 1);
	ddz = 1.0f / (dz - 1);

	dddx = (max[0] - min[0]) / (dx - 1);
	dddy = (max[1] - min[1]) / (dy - 1);
	dddz = (max[2] - min[2]) / (dz - 1);

	if (dz == 1)
	{
		ddz = 0;
		dddz = 0;
	}

	diag = max - min;
}

//! Zeros out the memory
void Volume::zeroOutMemory()
{
	for (uint i1 = 0; i1 < dx * dy * dz; i1++)
		// vol[i1] = Voxel(std::numeric_limits<float>::max(), 0.0f, Vector4uc{0, 0, 0, 0});
		vol[i1] = Voxel(1.0f, 0.0f, Vector4uc{0, 0, 0, 0});
}

//! Returns the Data.
Voxel *Volume::getData()
{
	return vol;
};

//! Sets new min and max points
void Volume::setNewBoundingPoints(Vector3f& min_, Vector3f& max_)
{
	min = min_;
	max = max_;
	zeroOutMemory();
	compute_ddx_dddx();
}

//! Updates the color of a voxel
void Volume::updateColor(Vector3i voxelCoords, Vector4uc& color, bool notVisited) {
	float weight = 1.0;
	//std::cout << voxelCoords << std::endl;
	Voxel& vox = get(voxelCoords[0], voxelCoords[1], voxelCoords[2]);

	if (notVisited)
		vox.setColor(color);
	else
		vox.setColor((vox.getColor() + color) / 2);
}

//! Updates the color of a voxel for a point p in grid coordinates
void Volume::updateColor(Vector3f point, Vector4uc& color, bool notVisited) {
	Vector3i p_int = Volume::intCoords(point);

	updateColor(Vector3i{ p_int[0] + 0, p_int[1] + 0, p_int[2] + 0 }, color, notVisited);
	updateColor(Vector3i{ p_int[0] + 1, p_int[1] + 0, p_int[2] + 0 }, color, notVisited);
	updateColor(Vector3i{ p_int[0] + 0, p_int[1] + 1, p_int[2] + 0 }, color, notVisited);
	updateColor(Vector3i{ p_int[0] + 1, p_int[1] + 1, p_int[2] + 0 }, color, notVisited);
	updateColor(Vector3i{ p_int[0] + 0, p_int[1] + 0, p_int[2] + 1 }, color, notVisited);
	updateColor(Vector3i{ p_int[0] + 1, p_int[1] + 0, p_int[2] + 1 }, color, notVisited);
	updateColor(Vector3i{ p_int[0] + 0, p_int[1] + 1, p_int[2] + 1 }, color, notVisited);
	updateColor(Vector3i{ p_int[0] + 1, p_int[1] + 1, p_int[2] + 1 }, color, notVisited);
}

// estimate the normal for a point in voxel grid coordinates using voxel grid by calculating the numerical derivative of TSDF
Vector3f Volume::calculateNormal(const Vector3f &point)
{
	//Vector3f shiftedXup, shiftedXdown, shiftedYup, shiftedYdown, shiftedZup, shiftedZdown;
	Vector3f shiftedXup, shiftedYup, shiftedZup;
	float x_dir, y_dir, z_dir;
	Vector3f normal;

	shiftedXup = point;
	shiftedXup[0] += 1;
	//shiftedXdown = point;
	//shiftedXdown[0] -= 1;

	shiftedYup = point;
	shiftedYup[1] += 1;
	//shiftedYdown = point;
	//shiftedYdown[1] -= 1;

	shiftedZup = point;
	shiftedZup[2] += 1;
	//shiftedZdown = point;
	//shiftedZdown[2] -= 1;

	float sdfXup = trilinearInterpolation(shiftedXup);
	//float sdfXdown = trilinearInterpolation(shiftedXdown);

	float sdfYup = trilinearInterpolation(shiftedYup);
	//float sdfYdown = trilinearInterpolation(shiftedYdown);

	float sdfZup = trilinearInterpolation(shiftedYup);
	//float sdfZdown = trilinearInterpolation(shiftedYdown);

	float sdfPoint = trilinearInterpolation(point);

	if (
		sdfXup == std::numeric_limits<float>::max() ||
		sdfYup == std::numeric_limits<float>::max() ||
		sdfZup == std::numeric_limits<float>::max() ||
		sdfPoint == std::numeric_limits<float>::max()
		)
		return Vector3f(MINF, MINF, MINF);

	x_dir = (sdfXup - sdfPoint) / (dddx);
	y_dir = (sdfYup - sdfPoint) / (dddy);
	z_dir = (sdfZup - sdfPoint) / (dddz);

	normal = Vector3f{ x_dir, y_dir, z_dir };
	normal.normalize();

	return normal;
}

float Volume::trilinearInterpolation(const Vector3f &p) const
{
	vector4f tmp;
	for (auto i = 0; i < 3; ++i)
		tmp[i] = p(i);
	trilinearInterpolation(tmp);
}

// trilinear interpolation of a point in voxel grid coordinates to get SDF at the point
float Volume::trilinearInterpolation(const vector4f &p) const
{
	Vector3i start = {(int)p[0], (int)p[1], (int)p[2]};
	float c000, c001, c010, c011, c100, c101, c110, c111;

	c000 = get(start[0] + 0, start[1] + 0, start[2] + 0).getTSDF();
	c100 = get(start[0] + 1, start[1] + 0, start[2] + 0).getTSDF();
	c001 = get(start[0] + 0, start[1] + 0, start[2] + 1).getTSDF();
	c101 = get(start[0] + 1, start[1] + 0, start[2] + 1).getTSDF();
	c010 = get(start[0] + 0, start[1] + 1, start[2] + 0).getTSDF();
	c110 = get(start[0] + 1, start[1] + 1, start[2] + 0).getTSDF();
	c011 = get(start[0] + 0, start[1] + 1, start[2] + 1).getTSDF();
	c111 = get(start[0] + 1, start[1] + 1, start[2] + 1).getTSDF();

#if 0
	if (
		c000 == std::numeric_limits<float>::max() || 
		c001 == std::numeric_limits<float>::max() || 
		c010 == std::numeric_limits<float>::max() || 
		c011 == std::numeric_limits<float>::max() ||
		c100 == std::numeric_limits<float>::max() ||
		c101 == std::numeric_limits<float>::max() ||
		c110 == std::numeric_limits<float>::max() ||
		c111 == std::numeric_limits<float>::max()
	)
		return std::numeric_limits<float>::max();
#endif

	float xd, yd, zd;

	xd = p[0] - start[0]; //(p[0] - start[0]) / (start[0] + 1 - start[0]);
	yd = p[1] - start[1]; //(p[1] - start[1]) / (start[1] + 1 - start[1]);
	zd = p[2] - start[2]; //(p[1] - start[2]) / (start[2] + 1 - start[2]);

	float c00, c01, c10, c11;

	c00 = c000 * (1 - xd) + c100 * xd;
	c01 = c001 * (1 - xd) + c101 * xd;
	c10 = c010 * (1 - xd) + c110 * xd;
	c11 = c011 * (1 - xd) + c111 * xd;

	float c0, c1;

	c0 = c00 * (1 - yd) + c10 * yd;
	c1 = c01 * (1 - yd) + c11 * yd;

	float c;

	c = c0 * (1 - zd) + c1 * zd;

	return c;
}

static matrix4f getRotation(
	const Eigen::Matrix4f &mat)
{
    matrix4f r;
    for (auto i = 0; i < 3; ++i)
        for (auto j = 0; j < 3; ++j)
            r[i][j] = mat(i, j);
    return r;
}

static vector4f getTranslation(
    const Eigen::Matrix4f &mat)
{
    vector4f t;
    t[0] = mat(0, 3);
    t[1] = mat(1, 3);
    t[2] = mat(2, 3);
    t[3] = 0;
    return t;
}

inline matrix4f convertToArray(
    const Eigen::Matrix3f &m)
{
    matrix4f rv = {};
    for (auto i = 0; i < 3; ++i)
        for (auto j = 0; j < 3; ++j)
            rv[i][j] = m.data()[j * 3 + i];
    return rv;
}

inline vector4f convertToArray(
	const Eigen::Vector3f &v)
{
    vector4f rv;
    rv[0] = v[0];
    rv[1] = v[1];
    rv[2] = v[2];
    rv[3] = 0;
    return rv;
}

inline vector4i round(const vector4f &v)
{
    auto vv = _mm_load_ps((const float*)&v);
    auto rrv = _mm_cvtps_epi32(vv);
    vector4i rv;
    _mm_store_si128( (__m128i*)&rv[0], rrv);
    return rv;
}

// using given frame calculate TSDF values for all voxels in the grid
// __attribute__((optimize("O0")))
void Volume::integrate(Frame frame)
{
	const Matrix4f worldToCamera = frame.getExtrinsicMatrix();
	const Matrix4f cameraToWorld = worldToCamera.inverse();
	const Matrix3f intrinsic = frame.getIntrinsicMatrix();
	auto translation = getTranslation(cameraToWorld); 
	const float *depthMap = frame.getDepthMap();
	const BYTE *colorMap = frame.getColorMap();
	int width = frame.getFrameWidth();
	int height = frame.getFrameHeight();

	// std::cout << intrinsic << std::endl;
					int cnt = 0, cnt2 = 0;


	// subscripts: g - global coordinate system | c - camera coordinate system | i - image space
	// short notations: V - vector | P - point | sdf - signed distance field value | tsdf - truncated sdf
	Vector3f Pg, Pc, ray, normal;
	Vector2i Pi;
	Vector4uc color;
	float depth, lambda, sdf, tsdf_weight, tsdf, weight, cos_angle;
	uint index;

	// std::cout << "Integrate starting..." << std::endl;

	auto ex_rotation = getRotation(frame.getExtrinsicMatrix());
	auto ex_translation = getTranslation(frame.getExtrinsicMatrix());
	auto in = convertToArray(frame.getIntrinsicMatrix());

	for (int i = 0; i < dx; i++)
	{
		for (int j = 0; j < dy; j++)
		{
			for (int k = 0; k < dz; k++)
			{
				vector4f p = {i, j, k, 0};

				// project the grid point into image space
				auto pg = gridToWorld(p);
				auto pc = ex_rotation * pg + ex_translation;
				auto pi = in * pc;
				for (int i = 0; i < 4; ++i)
					pi[i] /= pi[2];

				// std::cout << Pg << std::endl << Pc << std::endl << Pi << std::endl;

				// Pg = gridToWorld(i, j, k);
				// Pc = Frame::transformPoint(Pg, worldToCamera);
				// Pi = Frame::perspectiveProjection(Pc, intrinsic);

				// std::cout << Pg << std::endl << Pc << std::endl << Pi << std::endl;
				auto pii = round(pi);
				// auto pii = pi;
				auto x = pi[0] = pii[0];
				auto y = pi[1] = pii[1];

				if (x >= 0 && x < 640 && y >= 0 && y < 480)
				{
					// get the depth of the point
					index = pi[1] * width + pi[0];
					depth = depthMap[index];

					// calculate the sdf value
					lambda = (pc / pc[2]).norm();
					sdf = depth - ((pg - translation) / lambda).norm();

					// get the previous value and weight
					tsdf = vol[getPosFromTuple(i, j, k)].getTSDF();
					weight = vol[getPosFromTuple(i, j, k)].getWeight();
					color = vol[getPosFromTuple(i, j, k)].getColor();

					if (sdf >= -TRUNCATION && depth != MINF)
					{
						float current_tsdf = std::min(1.0f, sdf / TRUNCATION);
						float current_weight = 1.0f;
						float old_tsdf = vol[getPosFromTuple(i, j, k)].getTSDF();
						float old_weight = vol[getPosFromTuple(i, j, k)].getWeight();

						auto updated_tsdf = (old_weight * old_tsdf + current_weight * current_tsdf) / (old_weight + current_weight);
						auto updated_weight = old_weight + current_weight;

						vol[getPosFromTuple(i, j, k)].setTSDF(updated_tsdf);
						vol[getPosFromTuple(i, j, k)].setWeight(updated_weight);

						if (sdf <= TRUNCATION / 2 && sdf >= -TRUNCATION / 2)
						{
							vol[getPosFromTuple(i, j, k)].setColor(
								Vector4uc{(const unsigned char)((color[0] * old_weight + colorMap[4 * index + 0] * current_weight) / (old_weight + current_weight)),
										  (const unsigned char)((color[1] * old_weight + colorMap[4 * index + 1] * current_weight) / (old_weight + current_weight)),
										  (const unsigned char)((color[2] * old_weight + colorMap[4 * index + 2] * current_weight) / (old_weight + current_weight)),
										  (const unsigned char)((color[3] * old_weight + colorMap[4 * index + 3] * current_weight) / (old_weight + current_weight))});
						}
						++cnt;
					}
				}
			}
		}
	}

	// auto start = std::chrono::high_resolution_clock::now();
	// tree->build((Octree::Vox *)vol, 128);
	// auto stop = std::chrono::high_resolution_clock::now();
	// auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
	// std::cout << "### cnt = " << cnt << ", cnt2 " << cnt2 << std::endl;
	// assert(0);

	// std::cout << "Integrate done!" << std::endl;
}
