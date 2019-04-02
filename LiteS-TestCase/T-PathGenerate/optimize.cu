#include <cuda_runtime.h>
#include "optimize.h"

__global__
void populate_spherical_histogram(cacc::Vec3f view_pos, float max_distance,
	cacc::BVHTree<cacc::DEVICE>::Accessor const bvh_tree,
	cacc::PointCloud<cacc::DEVICE>::Data cloud,
	cacc::KDTree<3, cacc::DEVICE>::Accessor const kd_tree,
	cacc::Array<float, cacc::DEVICE>::Data sphere_hist)
{
	int const bx = blockIdx.x;
	int const tx = threadIdx.x;

	uint id = bx * blockDim.x + tx;

	if (id >= cloud.num_vertices) return;

	cacc::Vec3f v = cloud.vertices_ptr[id];
	cacc::Vec3f n = cloud.normals_ptr[id];
	cacc::Vec3f v2c = view_pos - v;
	float l = norm(v2c);

	float scale = 1.0f - (l / max_distance);
	if (scale <= 0.0f) return;

	cacc::Vec3f v2cn = v2c / l;
	float ctheta = dot(v2cn, n);
	// 0.087f ~ cos(85.0f / 180.0f * pi)
	if (ctheta < 0.087f) return;

	if (!visible(v, v2cn, l, bvh_tree)) return;

	float capture_difficulty = max(cloud.qualities_ptr[id], 0.0f);

	// 1.484f ~ 85.0f / 180.0f * pi
	float min_theta = min(cloud.values_ptr[id], 1.484f);

	float scaling = (pi / 2.0f) / ((pi / 2.0f) - min_theta);

	float theta = acosf(__saturatef(ctheta));
	float rel_theta = max(theta - min_theta, 0.0f) * scaling;
	float score = capture_difficulty * cosf(rel_theta) * scale;

	uint idx;
	cacc::nnsearch::find_nn<3u>(kd_tree, -v2cn, &idx, nullptr);
	atomicAdd(sphere_hist.data_ptr + idx, score);
}