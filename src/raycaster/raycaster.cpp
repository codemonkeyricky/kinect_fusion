#include <chrono>

#include "raycaster.h"

//RayCaster::RayCaster() {}

RayCaster::RayCaster(Volume& vol_) : vol(vol_) {}

RayCaster::RayCaster(Volume& vol_, Frame& frame_) : vol(vol_), frame(frame_) {}

void RayCaster::changeFrame(Frame& frame_) {
	frame = frame_;
}

void RayCaster::changeVolume(Volume& vol_) {
	vol = vol_;
}

// a function that writes down the invalid results
void mistake(
	std::vector<vector4f> &ovg, std::vector<Vector3f> &ong)
{
	vector4f error;
	error[0] = error[1] = error[2] = MINF;
	ovg.emplace_back(error);
	ong.emplace_back(Vector3f(MINF, MINF, MINF));
}

// __attribute__((optimize("O0")))
Frame &RayCaster::raycast()
{
	const Matrix4f worldToCamera = frame.getExtrinsicMatrix();
	const Matrix4f cameraToWorld = worldToCamera.inverse();
	const Matrix3f intrinsic_inverse = frame.getIntrinsicMatrix().inverse();
	Vector3f translation = cameraToWorld.block(0, 3, 3, 1);
	Matrix3f rotationMatrix = cameraToWorld.block(0, 0, 3, 3);
	vector4f rs, rn;
	Vector4uc color;

	int width = frame.getFrameWidth();
	int height = frame.getFrameHeight();

	Vector3f ray_start, ray_dir, ray_current, ray_previous, ray_next;

	std::shared_ptr<std::vector<vector4f>> output_vertices_global = std::make_shared<std::vector<vector4f>>(std::vector<vector4f>());
	output_vertices_global->reserve(width * height);

	std::shared_ptr<std::vector<Vector3f>> output_normals_global = std::make_shared<std::vector<Vector3f>>(std::vector<Vector3f>());
	output_normals_global->reserve(width * height);

	std::shared_ptr<std::vector<Vector3f>> output_vertices, output_normals;

	float sdf_1, sdf_2;
	Vector3f p, v, n;
	uint index;

	// std::cout << "RayCast starting..." << std::endl;
	int cnt = 0;

	auto t0 = std::chrono::high_resolution_clock::now();

	std::vector<std::array<vector4f, 2>> rays(640 * 480);

	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			// std::cout << i << " " << j << std::endl;

			// starting point is the position of the camera (translation) in grid coordinates
			rs = vol.worldToVoxel({translation[0], translation[1], translation[2], 0});
			for (auto i = 0; i < 3; ++i)
				ray_start[i] = rs[i];

			// calculate the direction vector as vector from camera position to the pixel(i, j)s world coordinates
			index = i * width + j;
			color = Vector4uc{frame.colorMap[4 * index + 0],
							  frame.colorMap[4 * index + 1],
							  frame.colorMap[4 * index + 2],
							  frame.colorMap[4 * index + 3]};
			/*
			ray_dir = Vector3f{ float(j), float(i), 1.0f };
			ray_dir = intrinsic_inverse * ray_dir;
			ray_dir = rotationMatrix * ray_dir;
			ray_dir = ray_dir.normalized();
			*/
			// ray_next = vol.worldToVoxel(frame.getVertexGlobal(index));

			ray_next = Vector3f{float(j), float(i), 1.0f};
			ray_next = intrinsic_inverse * ray_next;
			ray_next = rotationMatrix * ray_next + translation;
			rn = vol.worldToVoxel({ray_next[0], ray_next[1], ray_next[2], 0});
			for (auto i = 0; i < 3; ++i)
				ray_next[i] = rn[i];

			ray_dir = ray_next - ray_start;
			ray_dir = ray_dir.normalized();

			if (!ray_dir.allFinite() || ray_dir == Vector3f{0.0f, 0.0f, 0.0f})
			{
				mistake(*output_vertices_global, *output_normals_global);
				continue;
			}

			rays[i * 640 + j][0] = {ray_start(0), ray_start(1), ray_start(2), 0};
			rays[i * 640 + j][1] = {ray_dir(0), ray_dir(1), ray_dir(2), 0};
		}
	}

	auto t1 = std::chrono::high_resolution_clock::now();

	for (auto i = 0; i < rays.size(); ++i)
	{
		vector4f ray_current, ray_previous, p, v, n;
		{
			vector4f ray_start = rays[i][0];
			vector4f ray_dir = rays[i][1];

			ray_current = ray_start;

			if (!vol.isPointInVolume(ray_current))
			{
				mistake(*output_vertices_global, *output_normals_global);
				continue;
			}

			while (true)
			{
				ray_previous = ray_current;

				ray_current += ray_dir;

				if (!vol.isInterpolationPossible(ray_previous) || !vol.isInterpolationPossible(ray_current))
				{
					mistake(*output_vertices_global, *output_normals_global);
					break;
				}

				if (vol.get(ray_current).getTSDF() < 0)
				{
					sdf_1 = vol.trilinearInterpolation(ray_previous);
					sdf_2 = vol.trilinearInterpolation(ray_current);

					p = ray_previous - (ray_dir * sdf_1) / (sdf_2 - sdf_1);

					if (!vol.isInterpolationPossible(p))
					{
						mistake(*output_vertices_global, *output_normals_global);
						break;
					}

					// std::cout << ray_previous << std::endl << ray_current << std::endl << ray_dir << std::endl << sdf_1 << " " << sdf_2 << std::endl << p << std::endl;
					v = vol.voxelToWorld(p);
					// n = vol.calculateNormal(p);
					/*
					if (n == Vector3f{ MINF, MINF, MINF }) {
						mistake(*output_vertices_global, *output_normals_global);
						break;
					}
					*/
					output_vertices_global->emplace_back(v);
					// output_normals_global->emplace_back(n);

					if (!vol.voxelVisited(ray_previous))
					{
						vol.setVisited(ray_previous);
					}

					if (!vol.voxelVisited(ray_current))
						vol.setVisited(ray_current);

					// std::cout << p << std::endl;
					// if(!vol.voxelVisited(p))
					//	vol.updateColor(p, color, true);
					// else
					//	vol.updateColor(p, color, false);

					++cnt;

					break;
				}
				else if (vol.get(ray_current).getTSDF() == 0)
				{
					v = vol.voxelToWorld(ray_current);
					// n = vol.calculateNormal(ray_current);
					/*
					if (n == Vector3f{ MINF, MINF, MINF }) {
						mistake(*output_vertices_global, *output_normals_global);
						break;
					}
					*/
					output_vertices_global->emplace_back(v);
					// output_normals_global->emplace_back(n);

					if (!vol.voxelVisited(ray_current))
					{
						// vol.updateColor(ray_previous_int, color, true);
						vol.setVisited(ray_current);
					}

					break;
				}
			}
		}
	}

	auto t2 = std::chrono::high_resolution_clock::now();

	auto d1 = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0);
	auto d2 = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
	std::cout << "### raycast xform: " << d1.count() << " us" << std::endl;
	std::cout << "### raycast raycast: " << d2.count() << " us" << std::endl;

	// std::cout << "### raycast cnt: " << cnt << std::endl;

	// TODO: update _vector4f variants too

	// frame.mVerticesGlobal = output_vertices_global;
	// for (auto k = 0; k < 640 * 480; ++k)
	// 	for (auto i = 0; i < 3; ++i)
	// 		(*frame.mVerticesGlobal_vector4f)[k][i] = (*frame.mVerticesGlobal)[k](i);

	frame.mVerticesGlobal_vector4f = output_vertices_global;

	frame.mVerticesGlobal->resize(640 * 480);
	for (auto k = 0; k < 640 * 480; ++k)
		for (auto i = 0; i < 3; ++i)
			(*frame.mVerticesGlobal)[k](i) = (*frame.mVerticesGlobal_vector4f)[k][i];

	// std::shared_ptr<std::vector<vector4f>> mVerticesGlobal_vector4f;
	// std::shared_ptr<std::vector<vector4f>> mNormalGlobal_vector4f;

	// frame.mNormalsGlobal = output_normals_global;
	frame.mVertices = std::make_shared<std::vector<Vector3f>>(frame.transformPoints(*frame.mVerticesGlobal, worldToCamera));
	frame.computeNormalMap(width, height);
	frame.mNormalsGlobal = std::make_shared<std::vector<Vector3f>>(frame.rotatePoints(frame.getNormalMap(), rotationMatrix));

	for (auto k = 0; k < 640 * 480; ++k)
		for (auto i = 0; i < 3; ++i)
			(*frame.mNormalGlobal_vector4f)[k][i] = (*frame.mNormalsGlobal)[k](i);

	return frame;
}

// Vector3f ray_start = rays[i][0];
// Vector3f ray_dir = rays[i][1];
// Ray ray = Ray(ray_start, ray_dir);
// ray_curr = ray_start;

// int len = 32;
// while (true)
// {
//     // find a size that fits within bound
//     Vector3f mmin, mmax;
//     while (true)
//     {
//         auto mmin = Vector3f{ray_curr[0] - len, ray_curr[1] - len, ray_curr[2] - len};
//         auto mmax = Vector3f{ray_curr[0] + len, ray_curr[1] + len, ray_curr[2] + len};
//         if (len && (!vol.isInterpolationPossible(mmin) || !vol.isInterpolationPossible(mmax)))
//             len /= 2;
//         else
//             break;
//     }

//     // find a size that fits within bound
//     while (len && vol.tree->query_min(ray_curr[0] - len, ray_curr[0] + len,
//                                       ray_curr[1] - len, ray_curr[1] + len,
//                                       ray_curr[2] - len, ray_curr[2] + len) == 0)
//         len /= 2;

//     if (len == 0)
//     {
//         break;
//         Vector3i ray_prev_int = Vector3i{ray_prev(0), ray_prev(1), ray_prev(2)};
//         Vector3i ray_curr_int = Vector3i{ray_curr(0), ray_curr(1), ray_curr(2)};
//         if (vol.get(ray_prev_int).getTSDF() > 0 && vol.get(ray_curr_int).getTSDF() < 0)
//             break;
//     }

//     auto r = ray.next(std::max(1, len));
//     ray.setCurrPos(r);

//     ray_prev = ray_curr;
//     ray_curr = r;
// }