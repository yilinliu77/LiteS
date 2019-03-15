#include "CMesh.h"

using namespace std;

GLuint CMesh::boundIndex[36] = {
	0, 1, 2, 0, 2, 3, 0, 5, 1, 0, 6, 5, 0, 3, 7, 0, 7, 6, 4, 5, 6, 4, 6, 7, 4, 2, 3, 4, 3, 7, 4, 5, 1, 4, 1, 2
};

CMesh::CMesh(){}

bool CMesh::Intersect(Ray& ray, SurfaceInteraction* isect) {
	for (int i = 0; i < vertices.size() / 3; ++i) {
		// Get triangle vertices in _p0_, _p1_, and _p2_
		const glm::vec3& p0 = vertices[i * 3 + 0].Position;
		const glm::vec3& p1 = vertices[i * 3 + 1].Position;
		const glm::vec3& p2 = vertices[i * 3 + 2].Position;

		// Perform ray--triangle intersection test

		// Transform triangle vertices to ray coordinate space

		// Translate vertices based on ray origin
		glm::vec3 p0t = p0 - glm::vec3(ray.o);
		glm::vec3 p1t = p1 - glm::vec3(ray.o);
		glm::vec3 p2t = p2 - glm::vec3(ray.o);

		// Permute components of triangle vertices and ray direction
		int kz = maxDimension(abs(ray.d));
		int kx = kz + 1;
		if (kx == 3) kx = 0;
		int ky = kx + 1;
		if (ky == 3) ky = 0;
		glm::vec3 d = glm::vec3(ray.d[kx], ray.d[ky], ray.d[kz]);
		p0t = glm::vec3(p0t[kx], p0t[ky], p0t[kz]);
		p1t = glm::vec3(p1t[kx], p1t[ky], p1t[kz]);
		p2t = glm::vec3(p2t[kx], p2t[ky], p2t[kz]);

		// Apply shear transformation to translated vertex positions
		float Sx = -d[0] / d[2];
		float Sy = -d[1] / d[2];
		float Sz = 1.f / d[2];
		p0t[0] += Sx * p0t[2];
		p0t[1] += Sy * p0t[2];
		p1t[0] += Sx * p1t[2];
		p1t[1] += Sy * p1t[2];
		p2t[0] += Sx * p2t[2];
		p2t[1] += Sy * p2t[2];

		// Compute edge function coefficients _e0_, _e1_, and _e2_
		float e0 = p1t[0] * p2t[1] - p1t[1] * p2t[0];
		float e1 = p2t[0] * p0t[1] - p2t[1] * p0t[0];
		float e2 = p0t[0] * p1t[1] - p0t[1] * p1t[0];

		// Fall back to double precision test at triangle edges
		if (sizeof(float) == sizeof(float) &&
			(e0 == 0.0f || e1 == 0.0f || e2 == 0.0f)) {
			double p2txp1ty = (double)p2t[0] * (double)p1t[1];
			double p2typ1tx = (double)p2t[1] * (double)p1t[0];
			e0 = (float)(p2typ1tx - p2txp1ty);
			double p0txp2ty = (double)p0t[0] * (double)p2t[1];
			double p0typ2tx = (double)p0t[1] * (double)p2t[0];
			e1 = (float)(p0typ2tx - p0txp2ty);
			double p1txp0ty = (double)p1t[0] * (double)p0t[1];
			double p1typ0tx = (double)p1t[1] * (double)p0t[0];
			e2 = (float)(p1typ0tx - p1txp0ty);
		}

		// Perform triangle edge and determinant tests
		if ((e0 < 0 || e1 < 0 || e2 < 0) && (e0 > 0 || e1 > 0 || e2 > 0))
			return false;
		float det = e0 + e1 + e2;
		if (det == 0) return false;

		// Compute scaled hit distance to triangle and test against ray $t$ range
		p0t[2] *= Sz;
		p1t[2] *= Sz;
		p2t[2] *= Sz;
		float tScaled = e0 * p0t[2] + e1 * p1t[2] + e2 * p2t[2];
		if (det < 0 && (tScaled >= 0 || tScaled < ray.tMax * det))
			return false;
		else if (det > 0 && (tScaled <= 0 || tScaled > ray.tMax * det))
			return false;

		// Compute barycentric coordinates and $t$ value for triangle intersection
		float invDet = 1 / det;
		float b0 = e0 * invDet;
		float b1 = e1 * invDet;
		float b2 = e2 * invDet;
		float t = tScaled * invDet;

		// Ensure that computed triangle $t$ is conservatively greater than zero

		// Compute $\delta_z$ term for triangle $t$ error bounds
		float maxZt = maxComponent(abs(glm::vec3(p0t[2], p1t[2], p2t[2])));
		float deltaZ = gamma(3) * maxZt;

		// Compute $\delta_x$ and $\delta_y$ terms for triangle $t$ error bounds
		float maxXt = maxComponent(abs(glm::vec3(p0t[0], p1t[0], p2t[0])));
		float maxYt = maxComponent(abs(glm::vec3(p0t[1], p1t[1], p2t[1])));
		float deltaX = gamma(5) * (maxXt + maxZt);
		float deltaY = gamma(5) * (maxYt + maxZt);

		// Compute $\delta_e$ term for triangle $t$ error bounds
		float deltaE =
			2 * (gamma(2) * maxXt * maxYt + deltaY * maxXt + deltaX * maxYt);

		// Compute $\delta_t$ term for triangle $t$ error bounds and check _t_
		float maxE = maxComponent(abs(glm::vec3(e0, e1, e2)));
		float deltaT = 3 *
			(gamma(3) * maxE * maxZt + deltaE * maxZt + deltaZ * maxE) *
			std::abs(invDet);
		if (t <= deltaT) return false;

		// Interpolate $(u,v)$ parametric coordinates and hit point
		glm::vec3 pHit = b0 * p0 + b1 * p1 + b2 * p2;

		// Fill in _SurfaceInteraction_ from triangle hit
		*isect = SurfaceInteraction(pHit, t);

		ray.tMax = t;

	}

	return true;
}

bool CMesh::IntersectP(const Ray& ray) {
	// Get triangle vertices in _p0_, _p1_, and _p2_
	for (int i = 0; i < vertices.size() / 3; ++i) {
		// Get triangle vertices in _p0_, _p1_, and _p2_
		const glm::vec3& p0 = vertices[i * 3 + 0].Position;
		const glm::vec3& p1 = vertices[i * 3 + 1].Position;
		const glm::vec3& p2 = vertices[i * 3 + 2].Position;

		// Perform ray--triangle intersection test

		// Transform triangle vertices to ray coordinate space

		// Translate vertices based on ray origin
		glm::vec3 p0t = p0 - glm::vec3(ray.o);
		glm::vec3 p1t = p1 - glm::vec3(ray.o);
		glm::vec3 p2t = p2 - glm::vec3(ray.o);

		// Permute components of triangle vertices and ray direction
		int kz = maxDimension(abs(ray.d));
		int kx = kz + 1;
		if (kx == 3) kx = 0;
		int ky = kx + 1;
		if (ky == 3) ky = 0;
		glm::vec3 d = glm::vec3(ray.d[kx], ray.d[ky], ray.d[kz]);
		p0t = glm::vec3(p0t[kx], p0t[ky], p0t[kz]);
		p1t = glm::vec3(p1t[kx], p1t[ky], p1t[kz]);
		p2t = glm::vec3(p2t[kx], p2t[ky], p2t[kz]);

		// Apply shear transformation to translated vertex positions
		float Sx = -d[0] / d[2];
		float Sy = -d[1] / d[2];
		float Sz = 1.f / d[2];
		p0t[0] += Sx * p0t[2];
		p0t[1] += Sy * p0t[2];
		p1t[0] += Sx * p1t[2];
		p1t[1] += Sy * p1t[2];
		p2t[0] += Sx * p2t[2];
		p2t[1] += Sy * p2t[2];

		// Compute edge function coefficients _e0_, _e1_, and _e2_
		float e0 = p1t[0] * p2t[1] - p1t[1] * p2t[0];
		float e1 = p2t[0] * p0t[1] - p2t[1] * p0t[0];
		float e2 = p0t[0] * p1t[1] - p0t[1] * p1t[0];

		// Fall back to double precision test at triangle edges
		if (sizeof(float) == sizeof(float) &&
			(e0 == 0.0f || e1 == 0.0f || e2 == 0.0f)) {
			double p2txp1ty = (double)p2t[0] * (double)p1t[1];
			double p2typ1tx = (double)p2t[1] * (double)p1t[0];
			e0 = (float)(p2typ1tx - p2txp1ty);
			double p0txp2ty = (double)p0t[0] * (double)p2t[1];
			double p0typ2tx = (double)p0t[1] * (double)p2t[0];
			e1 = (float)(p0typ2tx - p0txp2ty);
			double p1txp0ty = (double)p1t[0] * (double)p0t[1];
			double p1typ0tx = (double)p1t[1] * (double)p0t[0];
			e2 = (float)(p1typ0tx - p1txp0ty);
		}

		// Perform triangle edge and determinant tests
		if ((e0 < 0 || e1 < 0 || e2 < 0) && (e0 > 0 || e1 > 0 || e2 > 0))
			return false;
		float det = e0 + e1 + e2;
		if (det == 0) return false;

		// Compute scaled hit distance to triangle and test against ray $t$ range
		p0t[2] *= Sz;
		p1t[2] *= Sz;
		p2t[2] *= Sz;
		float tScaled = e0 * p0t[2] + e1 * p1t[2] + e2 * p2t[2];
		if (det < 0 && (tScaled >= 0 || tScaled < ray.tMax * det))
			return false;
		else if (det > 0 && (tScaled <= 0 || tScaled > ray.tMax * det))
			return false;

		// Compute barycentric coordinates and $t$ value for triangle intersection
		float invDet = 1 / det;
		float b0 = e0 * invDet;
		float b1 = e1 * invDet;
		float b2 = e2 * invDet;
		float t = tScaled * invDet;

		// Ensure that computed triangle $t$ is conservatively greater than zero

		// Compute $\delta_z$ term for triangle $t$ error bounds
		float maxZt = maxComponent(abs(glm::vec3(p0t[2], p1t[2], p2t[2])));
		float deltaZ = gamma(3) * maxZt;

		// Compute $\delta_x$ and $\delta_y$ terms for triangle $t$ error bounds
		float maxXt = maxComponent(abs(glm::vec3(p0t[0], p1t[0], p2t[0])));
		float maxYt = maxComponent(abs(glm::vec3(p0t[1], p1t[1], p2t[1])));
		float deltaX = gamma(5) * (maxXt + maxZt);
		float deltaY = gamma(5) * (maxYt + maxZt);

		// Compute $\delta_e$ term for triangle $t$ error bounds
		float deltaE =
			2 * (gamma(2) * maxXt * maxYt + deltaY * maxXt + deltaX * maxYt);

		// Compute $\delta_t$ term for triangle $t$ error bounds and check _t_
		float maxE = maxComponent(abs(glm::vec3(e0, e1, e2)));
		float deltaT = 3 *
			(gamma(3) * maxE * maxZt + deltaE * maxZt + deltaZ * maxE) *
			std::abs(invDet);
		if (t <= deltaT) return false;

		return true;
	}
}