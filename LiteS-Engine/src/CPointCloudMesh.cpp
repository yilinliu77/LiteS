#include "CPointCloudMesh.h"


CPointCloudMesh::CPointCloudMesh(const std::vector<Vertex>& vPoints
	, const vector<unsigned> &vIndices):CMesh() {
	this->vertices = vPoints;
	this->indices = vIndices;

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
}

CPointCloudMesh::CPointCloudMesh(const std::vector<Vertex>& vPoints) :CMesh()
{
	this->vertices = vPoints;

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
}

CPointCloudMesh::CPointCloudMesh(const std::vector<Vertex>& vPoints
	, const glm::vec3 vColor) :CMesh()
{
	this->vertices = vPoints;
	for (auto &v : vertices)
		v.Color = vColor;

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
}

CPointCloudMesh::CPointCloudMesh(const std::vector<Vertex>& vPoints
	, const glm::vec3 vColor,const float vPointSize) :CMesh(),pointSize(vPointSize)
{
	this->vertices = vPoints;
	for (auto &v : vertices)
		v.Color = vColor;

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
}

CPointCloudMesh::CPointCloudMesh(const std::string & vPath) :CMesh()
{
	Assimp::Importer importer;
	const aiScene* scene;

	scene = importer.ReadFile(vPath, NULL);
	// check for errors
	if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) // if is Not Zero
		throw string("ERROR::ASSIMP:: ") + importer.GetErrorString();

	processPointCloudNode(scene->mRootNode, scene);
}

void CPointCloudMesh::processPointCloudNode(aiNode *node, const aiScene *scene) {
	vector<Vertex> vertices;
	vector<unsigned int> indices;

	aiMesh* aiMesh = scene->mMeshes[node->mMeshes[0]];

	for (unsigned int i = 0; i < aiMesh->mNumVertices; i++) {
		Vertex vertex;
		glm::vec3 vector; // we declare a placeholder vector since assimp uses its own vector class that doesn't directly convert to glm's vec3 class so we transfer the data to this placeholder glm::vec3 first.
						  // positions
		vector.x = aiMesh->mVertices[i].x;
		vector.y = aiMesh->mVertices[i].y;
		vector.z = aiMesh->mVertices[i].z;
		vertex.Position = vector;
		// normals
		if (aiMesh->mNormals) {
			vector.x = aiMesh->mNormals[i].x;
			vector.y = aiMesh->mNormals[i].y;
			vector.z = aiMesh->mNormals[i].z;
			vertex.Normal = vector;
		}

		//if (mesh->mColors[0]) {
		//	vector.x = mesh->mColors[0][i][0];
		//	vector.y = mesh->mColors[0][i][1];
		//	vector.z = mesh->mColors[0][i][2];
		//	vertex.Color = vector;
		//}

		vertices.push_back(vertex);
	}
	// now wake through each of the mesh's faces (a face is a mesh its triangle) and retrieve the corresponding vertex indices.
	for (unsigned int i = 0; i < aiMesh->mNumFaces; i++) {
		aiFace face = aiMesh->mFaces[i];
		// retrieve all indices of the face and store them in the indices vector
		for (unsigned int j = 0; j < face.mNumIndices; j++)
			indices.push_back(face.mIndices[j]);
	}

	this->vertices = vertices;
	this->indices = indices;

	glm::vec3 pmin(INFINITY, INFINITY, INFINITY);
	glm::vec3 pmax(-INFINITY, -INFINITY, -INFINITY);
	for (int i = 0; i < this->vertices.size(); ++i) {
		pmin[0] = std::min(pmin[0], this->vertices[i].Position[0]);
		pmin[1] = std::min(pmin[1], this->vertices[i].Position[1]);
		pmin[2] = std::min(pmin[2], this->vertices[i].Position[2]);
		pmax[0] = std::max(pmax[0], this->vertices[i].Position[0]);
		pmax[1] = std::max(pmax[1], this->vertices[i].Position[1]);
		pmax[2] = std::max(pmax[2], this->vertices[i].Position[2]);
	}
	bounds = Bounds3f();
	bounds.pMin = pmin;
	bounds.pMax = pmax;

}

void CPointCloudMesh::setupMeshWithIndex() {
	// create buffers/arrays
	glGenVertexArrays(1, &VAO);
	glGenBuffers(1, &VBO);
	glGenBuffers(1, &EBO);

	glBindVertexArray(VAO);
	// load data into vertex buffers
	glBindBuffer(GL_ARRAY_BUFFER, VBO);

	glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), nullptr, GL_STATIC_DRAW);
	glBufferSubData(GL_ARRAY_BUFFER, 0, vertices.size() * sizeof(Vertex), &vertices[0]);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), &indices[0], GL_STATIC_DRAW);

	// set the vertex attribute pointers
	// vertex Positions
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)0);
	// vertex normals
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, Normal));
	// vertex texcoords
	glEnableVertexAttribArray(2);
	glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, TexCoords));
	// vertex color
	glEnableVertexAttribArray(3);
	glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex)
		, (void*)offsetof(Vertex, Color));
	
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

	glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex) , nullptr, GL_STATIC_DRAW);
	glBufferSubData(GL_ARRAY_BUFFER, 0, vertices.size() * sizeof(Vertex), &vertices[0]);

	// set the vertex attribute pointers
	// vertex Positions
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, Position));
	// vertex normals
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, Normal));
	// vertex texcoords
	glEnableVertexAttribArray(2);
	glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, TexCoords));
	// vertex color
	glEnableVertexAttribArray(3);
	glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex)
		, (void*)offsetof(Vertex, Color));

	glBindVertexArray(0);
}

void CPointCloudMesh::Draw(CShader* shader) {
	std::lock_guard<std::mutex> lock(m_VAOMutex);
	// draw point cloud
	glBindVertexArray(VAO);

	if (!pointsVertexChangeIndex.empty()) {
		for (int i = 0; i < pointsVertexChange.size(); ++i) {
			glBindBuffer(GL_ARRAY_BUFFER, VBO);
			glBufferSubData(GL_ARRAY_BUFFER, sizeof(Vertex) *pointsVertexChangeIndex[i]
				, sizeof(Vertex), &pointsVertexChange[i]);
		}
		pointsVertexChange.clear();
		pointsVertexChangeIndex.clear();
	}
	float nowPointSize;
	glGetFloatv(GL_POINT_SIZE, &nowPointSize);
	if (this->pointSize != -1)
		glPointSize(this->pointSize);
	shader->setBool("renderNormal", false);
	glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(vertices.size()));
	glBindVertexArray(0);

	glPointSize(nowPointSize);
}

void CPointCloudMesh::Draw(CShader* shader, glm::mat4& vModelMatrix) {}

void CPointCloudMesh::changePos(glm::vec3 vNewPos, unsigned aIndex){
	Vertex t;
	t.Position = vNewPos;
	t.Normal = this->vertices[aIndex].Normal;
	t.Color= this->vertices[aIndex].Color;
	this->changeVertex(t, aIndex);
}

void CPointCloudMesh::changeColor(glm::vec3 vNewColor, unsigned aIndex) {
	Vertex t;
	t.Position = this->vertices[aIndex].Position;
	t.Normal = this->vertices[aIndex].Normal;
	t.Color = vNewColor;
	this->changeVertex(t, aIndex);
}

void CPointCloudMesh::changeNormal(glm::vec3 vNewNormal, unsigned aIndex) {
	Vertex t;
	t.Position = this->vertices[aIndex].Position;
	t.Normal = vNewNormal;
	t.Color = this->vertices[aIndex].Color;
	this->changeVertex(t, aIndex);
}

void CPointCloudMesh::changeVertex(Vertex vVertexPosition, unsigned aIndex) {
	std::lock_guard<std::mutex> lock(m_VAOMutex);

	pointsVertexChange.push_back(vVertexPosition);
	pointsVertexChangeIndex.push_back(aIndex);
}