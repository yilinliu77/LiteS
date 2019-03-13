#include "CPathGenarateComponent.h"
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

CPathGenarateComponent::CPathGenarateComponent(CScene * vScene) :CPointCloudComponent(vScene){ 
	DisplayPass = this->m_Scene->m_Pass.at("display");
}

CPathGenarateComponent::~CPathGenarateComponent() = default;

void CPathGenarateComponent::generate_nadir() {
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

void CPathGenarateComponent::extraAlgorithm() {
	generate_nadir();


	return;
	
}

void CPathGenarateComponent::extraInit() {}


