#include "CPointCloudMesh.h"


CPointCloudMesh::CPointCloudMesh(const std::vector<Vertex>& vPoints, const vector<unsigned> &vIndices) {
	this->vertices = vPoints;
	this->indices = vIndices;
	pointsColor.assign(vertices.size(), glm::vec3(0.0f, 0.0f, 0.0f));

	glm::vec3 pmin(INFINITY, INFINITY, INFINITY);
	glm::vec3 pmax(-INFINITY, -INFINITY, -INFINITY);
	for (int i = 0; i < vPoints.size(); ++i) {
		pmin[0] = std::min(pmin[0], vPoints[i].Position[0]);
		pmin[1] = std::min(pmin[1], vPoints[i].Position[1]);
		pmin[2] = std::min(pmin[2], vPoints[i].Position[2]);
		pmax[0] = std::max(pmax[0], vPoints[i].Position[0]);
		pmax[1] = std::max(pmax[1], vPoints[i].Position[1]);
		pmax[2] = std::max(pmax[2], vPoints[i].Position[2]);
	}
	bounds = Bounds3f();
	bounds.pMin = pmin;
	bounds.pMax = pmax;

	setupMeshWithIndex();
}

CPointCloudMesh::CPointCloudMesh(const std::vector<Vertex>& vPoints)
{
	this->vertices = vPoints;
	pointsColor.assign(vertices.size(), glm::vec3(0.0f, 0.0f, 0.0f));

	glm::vec3 pmin(INFINITY, INFINITY, INFINITY);
	glm::vec3 pmax(-INFINITY, -INFINITY, -INFINITY);
	for (int i = 0; i < vPoints.size(); ++i) {
		pmin[0] = std::min(pmin[0], vPoints[i].Position[0]);
		pmin[1] = std::min(pmin[1], vPoints[i].Position[1]);
		pmin[2] = std::min(pmin[2], vPoints[i].Position[2]);
		pmax[0] = std::max(pmax[0], vPoints[i].Position[0]);
		pmax[1] = std::max(pmax[1], vPoints[i].Position[1]);
		pmax[2] = std::max(pmax[2], vPoints[i].Position[2]);
	}
	bounds = Bounds3f();
	bounds.pMin = pmin;
	bounds.pMax = pmax;

	setupMesh();
}

void CPointCloudMesh::setupMeshWithIndex() {
	// create buffers/arrays
	glGenVertexArrays(1, &VAO);
	glGenBuffers(1, &VBO);
	glGenBuffers(1, &EBO);

	glBindVertexArray(VAO);
	// load data into vertex buffers
	glBindBuffer(GL_ARRAY_BUFFER, VBO);

	glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex)+pointsColor.size()*sizeof(glm::vec3), nullptr, GL_STATIC_DRAW);
	glBufferSubData(GL_ARRAY_BUFFER, 0, vertices.size() * sizeof(Vertex), &vertices[0]);
	glBufferSubData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex)
		, pointsColor.size() * sizeof(glm::vec3), &pointsColor[0]);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), &indices[0], GL_STATIC_DRAW);

	// set the vertex attribute pointers
	// vertex Positions
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)0);
	// vertex normals
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, Normal));
	// vertex texcoords
	glEnableVertexAttribArray(2);
	glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, Normal));
	// vertex color
	glEnableVertexAttribArray(3);
	glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, sizeof(glm::vec3)
		, (void*)(vertices.size() * sizeof(Vertex)));
	
	glBindVertexArray(0);
}

void CPointCloudMesh::setupMesh()
{
	// create buffers/arrays
	glGenVertexArrays(1, &VAO);
	glGenBuffers(1, &VBO);
	glGenBuffers(1, &EBO);

	glBindVertexArray(VAO);
	// load data into vertex buffers
	glBindBuffer(GL_ARRAY_BUFFER, VBO);

	glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex) + pointsColor.size() * sizeof(glm::vec3), nullptr, GL_STATIC_DRAW);
	glBufferSubData(GL_ARRAY_BUFFER, 0, vertices.size() * sizeof(Vertex), &vertices[0]);
	glBufferSubData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex)
		, pointsColor.size() * sizeof(glm::vec3), &pointsColor[0]);

	// set the vertex attribute pointers
	// vertex Positions
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)0);
	// vertex normals
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, Normal));
	// vertex texcoords
	glEnableVertexAttribArray(2);
	glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, Normal));
	// vertex color
	glEnableVertexAttribArray(3);
	glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, sizeof(glm::vec3)
		, (void*)(vertices.size() * sizeof(Vertex)));

	glBindVertexArray(0);
}

void CPointCloudMesh::Draw(CShader* shader) {
	std::lock_guard<std::mutex> lock(m_VAOMutex);
	// draw point cloud
	glBindVertexArray(VAO);

	if(!pointsIndexAdd.empty()) {
		for (int i = 0; i < pointsColorAdd.size();++i) {
			glBufferSubData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex) + sizeof(glm::vec3)*pointsIndexAdd[i]
				, sizeof(glm::vec3), &pointsColorAdd[i]);
		}
		pointsColorAdd.clear();
		pointsIndexAdd.clear();
	}

	glDrawArrays(GL_POINTS,0, vertices.size());
	glBindVertexArray(0);

}

void CPointCloudMesh::Draw(CShader* shader, glm::mat4& vModelMatrix) {}

void CPointCloudMesh::changeColor(glm::vec3 aColor, unsigned aIndex) {
	this->pointsColor[aIndex] = aColor;
	std::lock_guard<std::mutex> lock(m_VAOMutex);

	/*glBindVertexArray(VAO);
	glBindBuffer(GL_ARRAY_BUFFER, VBO);

	glBufferSubData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex) + sizeof(glm::vec3)*aIndex
		, sizeof(glm::vec3), &aColor[0]);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);*/

	pointsColorAdd.push_back(aColor);
	pointsIndexAdd.push_back(aIndex);
}