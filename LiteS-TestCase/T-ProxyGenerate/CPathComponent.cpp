#include "CPathComponent.h"
#include "CPointCloudMesh.h"
#include "CBVHACCEL.h"

#include "util.h"
#include <set>
#include <random>

#include <fssr/iso_surface.h>
#include <fssr/octree.h>
#include <mve/mesh.h>
#include <mve/defines.h>
#include <mve/mesh_io_ply.h>

const float insert_sample_density = 2;

CPathComponent::CPathComponent(CScene * vScene) :CPointCloudComponent(vScene){ 
	DisplayPass = this->m_Scene->m_Pass.at("display");
}

CPathComponent::~CPathComponent() = default;

void CPathComponent::sample_mesh(string vPath) {
	CMesh* mesh = DisplayPass->m_Models[0]->meshes[0];

	int samples_num = 4;

	vector<Vertex> &in_vertexs = mesh->vertices;
	vector<unsigned int> &in_indices = mesh->indices;
	//
	// Calculate total area
	//
	float total_surface_area = 0;
	for (size_t i=0;i<in_indices.size();i+=3)
	{
		total_surface_area += triangleArea(in_vertexs[in_indices[i]].Position
			, in_vertexs[in_indices[i + 1]].Position
			, in_vertexs[in_indices[i + 2]].Position);
	}

	std::vector<Vertex> out_samples;
	out_samples.reserve(static_cast<size_t>(samples_num*total_surface_area));
	//
	// Random sample point in each face of the mesh
	//
	std::uniform_real_distribution<float> dist(0.0f, 1.0f);
	for (size_t i = 0; i < in_indices.size(); i += 3)
	{
		glm::vec3 vertex0 = in_vertexs[in_indices[i]].Position;
		glm::vec3 vertex1 = in_vertexs[in_indices[i + 1]].Position;
		glm::vec3 vertex2 = in_vertexs[in_indices[i + 2]].Position;

		float item_surface_area = triangleArea(vertex0, vertex1, vertex2);
		size_t local_sample_num = static_cast<size_t>(item_surface_area * samples_num);

		std::mt19937 gen;
		gen.seed(static_cast<unsigned int>(i));

		glm::vec3 face_normal = glm::normalize(glm::cross(vertex2-vertex1, vertex2 - vertex0));

		for (size_t j=0;j< local_sample_num;++j)
		{
			float r1 = dist(gen);
			float r2 = dist(gen);

			float tmp = std::sqrt(r1);
			float u = 1.0f - tmp;
			float v = r2 * tmp;

			float w = 1.0f - v - u;

			Vertex vertex;
			vertex.Position = u * vertex0 + v * vertex1 + w * vertex2;
			vertex.Normal = face_normal;
			out_samples.push_back(vertex);
		}
	}

	CMesh* outMesh = new CPointCloudMesh(out_samples);
	saveMesh(outMesh, vPath);
}

template<int N>
void patch(My8BitRGBImage* img, int x, int y, float(*ptr)[N][N]) {
	for (int i = 0; i < N; ++i) {
		for (int j=0;j< N;++j)
		{
			(*ptr)[i][j] = img->data[(y - 1 + i)*img->ncols + (x - 1 + j)];
		}
	}
}

template<int N>
void averageFilter(My8BitRGBImage* img, float vBound) {
	for (size_t y = 0; y < img->nrows; ++y) {
		for (size_t x = 0; x < img->ncols; ++x) {
			if (y <= N || y >= img->nrows - N +1 || x <= N || x >= img->ncols - N +1) {
				img->data[y*img->ncols + x] = vBound;
				continue;
			}

			float heights[N*N];
			patch<N>(img, x, y, (float(*)[N][N])&heights);
			float sum = 0;
			for (size_t i = 0; i < N*N; i++)
			{
				sum += heights[i] > vBound ? heights[i] : 0;
			}
			img->data[y*img->ncols + x] = sum / N / N;
		}
	}
}

void CPathComponent::fixDiscontinuecy(string vPath){
	CMesh* outMesh = new CPointCloudMesh("C:/Users/vcc/Documents/repo/RENDERING/LiteS/point_after_sampling.ply");


	//
	// Generate height map
	//
	const float LOWEST = -99999.0f;
	float resolution = 0.5;
	int image_width = resolution * static_cast<int>(outMesh->bounds.pMax[0] - outMesh->bounds.pMin[0]+1);
	int image_height = resolution * static_cast<int>(outMesh->bounds.pMax[1] - outMesh->bounds.pMin[1]+1);
	My8BitRGBImage image;
	image.data = new float[image_height*image_width];
	image.ncols = image_width;
	image.nrows = image_height;

	for (size_t i=0;i<image_height*image_width;++i)
	{
		image.data[i] = LOWEST;
	}
	//
	// Fill the height map with height
	//
	for(size_t i=0;i< outMesh->vertices.size();++i)
	{
		int x = static_cast<int>((outMesh->vertices[i].Position.x - outMesh->bounds.pMin[0]) * resolution);
		int y = static_cast<int>((outMesh->vertices[i].Position.y - outMesh->bounds.pMin[1]) * resolution);
		if (image.data[y*image_width + x] < outMesh->vertices[i].Position.z)
			image.data[y*image_width + x] = outMesh->vertices[i].Position.z;
	}

	//
	//Median Filter to the height map
	//
	for (size_t y = 0; y < image_height; ++y) {
		for (size_t x = 0; x < image_width; ++x) {
			if (y <= 2 || y >= image_height - 3 || x <= 2 || x >= image_width - 3) {
				image.data[y*image_width + x] = LOWEST;
				continue;
			}

			float heights[16];
			patch<4>(&image, x, y, (float(*)[4][4])&heights);
			std::nth_element(heights, heights + (16 / 2), heights + 16);
			image.data[y*image_width + x] = heights[16 / 2];
		}
	}

	//averageFilter<4>(&image, LOWEST);

	/*bool holes = true;
	int hole_filling_iters = 50;
	while (holes && 0 < hole_filling_iters--) {
		holes = false;

		for (int y = 0; y < image_height; ++y) {
			for (int x = 0; x < image_width; ++x) {
				if (y <= 2 || y >= image_height - 3 || x <= 2 || x >= image_width - 3) {
					image.data[y*image_width + x] = LOWEST;
					continue;
				}
				else {
					float heights[9];
					patch<3>(&image, x, y, (float(*)[3][3])&heights);

					float * end = std::remove(heights, heights + 9, LOWEST);

					int n = std::distance(heights, end);

					if (n >= 3) {
						std::sort(heights, end);
						image.data[y*image_width + x] = heights[n / 2];
					}
					else {
						image.data[y*image_width + x] = LOWEST;
						holes = true;
					}
				}
			}
		}

	}*/

	//
	// Calculate ground
	//
	float ground_level = -LOWEST;
	for (size_t y = 0; y < image_height; ++y) {
		for (size_t x = 0; x < image_width; ++x) {
			if(image.data[y*image_width + x]==LOWEST) continue;
			ground_level = image.data[y*image_width + x] < ground_level
				? image.data[y*image_width + x] : ground_level;
		}
	}
	//
	// Scale the height map
	//
	for (size_t y = 0; y < image_height; ++y) {
		for (size_t x = 0; x < image_width; ++x) {
			if (image.data[y*image_width + x] != LOWEST)
				image.data[y*image_width + x] = (image.data[y*image_width + x] - ground_level);
			else
				image.data[y*image_width + x] = 0;

		}
	}

	//
	// Detect the height discontinuecy, generate new points
	//
	std::vector<Vertex> proxy_vertexes;

	for (size_t y = 0; y < image_height; ++y) {
		for (size_t x = 0; x < image_width; ++x) {
			if (y <= 1 || y >= image_height - 2 || x <= 1 || x >= image_width - 2) continue;

			float heights[3][3];
			patch(&image, x, y, &heights);

			
			float * it = (float *)heights;
			bool invalid = std::any_of(it, it + 9, [](float h) {
				return h < 0;
			});
			if (invalid) continue;
			

			float gy =(heights[0][0] - heights[2][0])
				+ 2.0f * (heights[0][1] - heights[2][1])
				+ (heights[0][2] - heights[2][2]);
			float gx =(heights[0][0] - heights[0][2])
				+ 2.0f * (heights[1][0] - heights[1][2])
				+ (heights[2][0] - heights[2][2]);

			//float gx = image.data[(y - 1)*image_width + x - 1] - image.data[(y + 1)*image_width + x - 1]
			//	+ 2 * (image.data[(y - 1)*image_width + x] - image.data[(y + 1)*image_width + x])
			//	+ image.data[(y - 1)*image_width + x + 1] - image.data[(y + 1)*image_width + x + 1];
			//float gy = image.data[(y - 1)*image_width + x - 1] - image.data[(y - 1)*image_width + x + 1]
			//	+ 2 * (image.data[(y)*image_width + x - 1] - image.data[(y)*image_width + x + 1])
			//	+ image.data[(y + 1)*image_width + x - 1] - image.data[(y + 1)*image_width + x + 1];
			
			float screen_x = x / resolution + outMesh->bounds.pMin[0];
			float screen_y = y / resolution + outMesh->bounds.pMin[1];
			
			Vertex t;
			t.Normal = glm::vec3(gx, gy, 1.0f);
			t.Normal = glm::normalize(t.Normal);
			t.Position = glm::vec3(screen_x, screen_y, image.data[y*image_width + x]);

			proxy_vertexes.push_back(t);

			float ldx = abs(-image.data[(y)*image_width + x - 1] + image.data[y*image_width + x]);
			float udy = abs(-image.data[(y - 1)*image_width + x] + image.data[y*image_width + x]);
			float rdx = abs(-image.data[y*image_width + x] + image.data[(y)*image_width + x + 1]);
			float ddy = abs(-image.data[y*image_width + x] + image.data[(y + 1)*image_width + x]);

			float m = std::max(std::max(ldx, rdx), std::max(udy, ddy));

			if (m * resolution < 1.0f) continue;


			for (int i=0;i< resolution * m;++i)
			{
				t.Normal = glm::vec3(gx, gy, 0.0f);
				t.Normal = glm::normalize(t.Normal);
				t.Position = glm::vec3(screen_x, screen_y, image.data[y*image_width + x] - float(i)/ resolution);
				if (t.Position.z<0)
				{
					break;
				}
				proxy_vertexes.push_back(t);
			}
		}
	}
	CMesh* proxy_points = new CPointCloudMesh(proxy_vertexes);
	saveMesh(proxy_points,vPath);
}

void CPathComponent::addSafeSpace(string vPath){
	CMesh* outMesh = new CPointCloudMesh("C:/Users/vcc/Documents/repo/RENDERING/LiteS/proxy_point.ply");



	saveMesh(outMesh,vPath);
}

void CPathComponent::reconstructMesh(string vPath){
	CMesh* outMesh = new CPointCloudMesh("C:/Users/vcc/Documents/repo/RENDERING/LiteS/proxy_point.ply");

	fssr::IsoOctree octree;
	
	for (auto vertex : outMesh->vertices) {
		fssr::Sample s;

		s.pos = math::Vec3d(vertex.Position.x, vertex.Position.y, vertex.Position.z);
		s.normal= math::Vec3d(vertex.Normal.x, vertex.Normal.y, vertex.Normal.z);
		s.scale = 1.0f;
		octree.insert_sample(s);
	}

	octree.limit_octree_level();
	octree.compute_voxels();
	octree.clear_samples();

	fssr::IsoSurface iso_surface(&octree, fssr::INTERPOLATION_CUBIC);
	mve::TriangleMesh::Ptr mve_mesh = iso_surface.extract_mesh();
	
	std::size_t num_vertices = mve_mesh->get_vertices().size();
	std::vector<float> confidences = mve_mesh->get_vertex_confidences();
	mve::TriangleMesh::DeleteList delete_verts(num_vertices, false);
	for (std::size_t i = 0; i < num_vertices; ++i) {
		if (confidences[i] == 0.0f) {
			delete_verts[i] = true;
		}
	}
	mve_mesh->delete_vertices_fix_faces(delete_verts);

	mve::geom::SavePLYOptions opts;
	opts.write_vertex_normals = true;
	mve::geom::save_ply_mesh(mve_mesh, vPath, opts);
}

void CPathComponent::extraAlgorithm() {
	//sample_mesh("C:/Users/vcc/Documents/repo/RENDERING/LiteS/point_after_sampling.ply");
	fixDiscontinuecy("C:/Users/vcc/Documents/repo/RENDERING/LiteS/proxy_point.ply");

	//addSafeSpace("C:/Users/vcc/Documents/repo/RENDERING/LiteS/proxy_airspace.ply");

	reconstructMesh("C:/Users/vcc/Documents/repo/RENDERING/LiteS/proxy_mesh.ply");

	return;
	
}

void CPathComponent::extraInit() {}


