#include "CPathComponent.h"
#include "CPointCloudMesh.h"

#include "util.h"
#include <set>
#include <random>

struct My8BitRGBImage
{
	int ncols;
	int nrows;
	float* data;

};

CPathComponent::CPathComponent(CScene * vScene) :CPointCloudComponent(vScene){ 
	DisplayPass = this->m_Scene->m_Pass.at("display");
}

CPathComponent::~CPathComponent() = default;

void CPathComponent::sample_mesh(string vPath) {
	CMesh* mesh = DisplayPass->m_Models[0]->meshes[0];

	int samples_num = 25;

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
	out_samples.reserve(samples_num*total_surface_area);
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
		size_t local_sample_num = item_surface_area * samples_num;

		std::mt19937 gen;
		gen.seed(i);

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

void CPathComponent::fixDiscontinuecy(string vPath){
	CMesh* outMesh = new CPointCloudMesh("C:/Users/vcc/Documents/repo/RENDERING/LiteS/point_after_sampling.ply");


	//
	// Generate height map
	//
	const float LOWEST = -99999.0f;
	int image_width = outMesh->bounds.pMax[0] - outMesh->bounds.pMin[0]+1;
	int image_height = outMesh->bounds.pMax[1] - outMesh->bounds.pMin[1]+1;
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
		int x = (outMesh->vertices[i].Position.x - outMesh->bounds.pMin[0]);
		int y = (outMesh->vertices[i].Position.y - outMesh->bounds.pMin[1]);
		if (image.data[y*image_width + x] < outMesh->vertices[i].Position.z)
			image.data[y*image_width + x] = outMesh->vertices[i].Position.z;
	}
	//
	// Calculate ground
	//
	float ground_level = -LOWEST;
	for (size_t y = 0; y < image_height; ++y) {
		for (size_t x = 0; x < image_width; ++x) {
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
		}
	}

	//
	// Detect the height discontinuecy, generate new points
	//
	std::vector<Vertex> proxy_vertexes;

	bool ground = true;
	for (size_t y = 0; y < image_height; ++y) {
		for (size_t x = 0; x < image_width; ++x) {
			if (y <= 1 || y >= image_height - 2 || x <= 1 || x >= image_width - 2) continue;

			float gx = image.data[(y - 1)*image_width + x - 1] - image.data[(y + 1)*image_width + x - 1]
				+ 2 * (image.data[(y - 1)*image_width + x] - image.data[(y + 1)*image_width + x])
				+ image.data[(y - 1)*image_width + x + 1] - image.data[(y + 1)*image_width + x + 1];
			float gy = image.data[(y - 1)*image_width + x - 1] - image.data[(y - 1)*image_width + x + 1]
				+ 2 * (image.data[(y)*image_width + x - 1] - image.data[(y)*image_width + x + 1])
				+ image.data[(y + 1)*image_width + x - 1] - image.data[(y + 1)*image_width + x + 1];
			
			float screen_x = x + outMesh->bounds.pMin[0];
			float screen_y = y + outMesh->bounds.pMin[1];
			
			Vertex t;
			t.Normal = glm::vec3(gx, gy, 1.0f);
			t.Normal = glm::normalize(t.Normal);
			t.Position = glm::vec3(screen_x, screen_y, ground ? 0 : image.data[y*image_width + x]);

			proxy_vertexes.push_back(t);

			float ldx = abs(-image.data[(y)*image_width + x - 1] + image.data[y*image_width + x]);
			float udy = abs(-image.data[(y - 1)*image_width + x] + image.data[y*image_width + x]);
			float rdx = abs(-image.data[y*image_width + x] + image.data[(y)*image_width + x + 1]);
			float ddy = abs(-image.data[y*image_width + x] + image.data[(y + 1)*image_width + x]);

			float m = std::max(std::max(ldx, rdx), std::max(udy, ddy));

			if (m < 1.5f) continue;
			ground = !ground;

			for (int i=0;i<m;++i)
			{
				t.Normal = glm::vec3(gx, gy, 0.0f);
				t.Normal = glm::normalize(t.Normal);
				t.Position = glm::vec3(screen_x, screen_y,i);
				proxy_vertexes.push_back(t);
			}
		}
	}
	CMesh* proxy_points = new CPointCloudMesh(proxy_vertexes);
	saveMesh(proxy_points,vPath);
}

void CPathComponent::addSafeSpace(string vPath){
	CMesh* outMesh = new CPointCloudMesh("C:/Users/vcc/Documents/repo/RENDERING/LiteS/point_after_sampling.ply");

	saveMesh(proxy_points,vPath);
}

void CPathComponent::reconstructMesh(string vPath){

}

void CPathComponent::extraAlgorithm() {
	//sample_mesh("C:/Users/vcc/Documents/repo/RENDERING/LiteS/point_after_sampling.ply");
	//fixDiscontinuecy("C:/Users/vcc/Documents/repo/RENDERING/LiteS/proxy_point.ply")

	addSafeSpace("C:/Users/vcc/Documents/repo/RENDERING/LiteS/proxy_airspace.ply")

	reconstructMesh("C:/Users/vcc/Documents/repo/RENDERING/LiteS/proxy_point.ply")

	return;
	
}

void CPathComponent::extraInit() {}


