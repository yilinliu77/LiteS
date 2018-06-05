#include "CMesh.h"

using namespace std;

CMesh::CMesh(string vType) {
	if (vType=="window"){
		const float WindowVertices[] = {
			//2 position,2 texture
			-1, -1, 0, 0, //0
			-1, 1, 0, 1, //2
			1, -1, 1, 0, //5
			1, 1, 1, 1 //7
		};

		const unsigned int WindowIndex[] = { 0,2,3, 0,3,1 };

		for (int i = 0; i < 6; ++i)
			this->indices.push_back(WindowIndex[i]);
		glGenVertexArrays(1, &VAO);
		glGenBuffers(1, &VBO);
		glGenBuffers(1, &EBO);

		glBindVertexArray(VAO);
		glBindBuffer(GL_ARRAY_BUFFER, VBO);
		glBufferData(GL_ARRAY_BUFFER, sizeof(WindowVertices), &WindowVertices[0], GL_STATIC_DRAW);

		/*glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(WindowIndex), &WindowIndex[0], GL_STATIC_DRAW);*/

		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), &indices[0], GL_STATIC_DRAW);

		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);

		glEnableVertexAttribArray(1);
		glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)(2 * sizeof(float)));

		glBindVertexArray(0);
	}
	

	
}

CMesh::CMesh() {

}

CMesh::CMesh(vector<Vertex> vertices, vector<unsigned int> indices
			 , MeshMaterial material, vector<Texture> textures) : model(glm::mat4(1.0)) {
	this->vertices = vertices;
	this->indices = indices;
	this->textures = textures;
	this->material = material;

	// now that we have all the required data, set the vertex buffers and its attribute pointers.
	setupMesh();

	//deal with bounds
	float minx = 99999999.0f, miny = 99999999.0f, minz = 99999999.0f;
	float maxx = -99999999.0f, maxy = -99999999.0f, maxz = -99999999.0f;
	int countTemp = vertices.size();
	for (int i = 0; i < countTemp; ++i) {
		minx = vertices[i].Position.x < minx ? vertices[i].Position.x : minx;
		miny = vertices[i].Position.y < miny ? vertices[i].Position.y : miny;
		minz = vertices[i].Position.z < minz ? vertices[i].Position.z : minz;
		maxx = vertices[i].Position.x > maxx ? vertices[i].Position.x : maxx;
		maxy = vertices[i].Position.y > maxy ? vertices[i].Position.y : maxy;
		maxz = vertices[i].Position.z > maxz ? vertices[i].Position.z : maxz;
	}
	this->bounds = Bounds3f(glm::vec3(minx, miny, minz), glm::vec3(maxx, maxy, maxz));
}

CMesh::CMesh(glm::vec3 c, float edge) {
	glm::vec3 boundVerticesVector[] = {
		c + glm::vec3(edge / 2, edge / 2, edge / 2),
		glm::vec3(1, 1, 1),
		c + glm::vec3(edge / 2, -edge / 2, edge / 2),
		glm::vec3(1, -1, 1),
		c + glm::vec3(edge / 2, -edge / 2, -edge / 2),
		glm::vec3(1, -1, -1),
		c + glm::vec3(edge / 2, edge / 2, -edge / 2),
		glm::vec3(1, 1, -1),
		c + glm::vec3(-edge / 2, -edge / 2, -edge / 2),
		glm::vec3(-1, -1, -1),
		c + glm::vec3(-edge / 2, -edge / 2, edge / 2),
		glm::vec3(-1, -1, 1),
		c + glm::vec3(-edge / 2, edge / 2, edge / 2),
		glm::vec3(-1, 1, 1),
		c + glm::vec3(-edge / 2, edge / 2, -edge / 2),
		glm::vec3(-1, 1, -1)
	};
	Vertex vertex;
	for (int i = 0; i < 8; ++i) {
		vertex.Position = boundVerticesVector[2 * i];
		vertex.Normal = boundVerticesVector[2 * i + 1];
		this->vertices.push_back(vertex);
	}
	for (int i = 0; i < 36; i++)
		this->indices.push_back(boundIndex[i]);
	this->material = MeshMaterial();
	this->material.diffuse = glm::vec3(0, 1, 0);
	this->bounds = Bounds3f(c + glm::vec3(-edge / 2, -edge / 2, -edge / 2)
							, c + glm::vec3(edge / 2, edge / 2, edge / 2));
	this->model = glm::translate(glm::mat4(1), c) * glm::scale(glm::mat4(1), glm::vec3(edge, edge, edge));

	//this->UseTexture = false;
	setupMesh();
}

void CMesh::Draw(CShader* shader) {

	if (this->textures.size() != 0) {
		shader->setBool("useTexture", true);

		unsigned int diffuseNr = 1;
		unsigned int specularNr = 1;
		for (unsigned int i = 0; i < textures.size(); i++) {
			glActiveTexture(GL_TEXTURE0 + i); // 在绑定之前激活相应的纹理单元
											  // 获取纹理序号（diffuse_textureN 中的 N）
			string number;
			string name = textures[i].type;
			if (name == "texture_diffuse")
				number = std::to_string(diffuseNr++);
			else if (name == "texture_specular")
				number = std::to_string(specularNr++);

			//shader->setInt((name + number).c_str(), i);
			glUniform1i(glGetUniformLocation(shader->ID, (name + number).c_str()), i);
			glBindTexture(GL_TEXTURE_2D, textures[i].id);
		}
		glActiveTexture(GL_TEXTURE0);

	}
	else {
		shader->setBool("useTexture", false);
		shader->setVec3("material.diffuse", this->material.diffuse);
		shader->setVec3("material.specular", this->material.specular);
		shader->setFloat("material.shininess", this->material.shininess);
	}

	
	// draw mesh
	glBindVertexArray(VAO);
	glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);

	glBindVertexArray(0);

}

void CMesh::Draw(CShader* shader,glm::mat4 &vModelMatrix) {
	shader->setMat4("model", vModelMatrix);

	shader->setBool("useTexture", false);

	if (this->textures.size() != 0) {
		shader->setBool("useTexture", true);

		unsigned int diffuseNr = 1;
		unsigned int specularNr = 1;
		for (unsigned int i = 0; i < textures.size(); i++) {
			glActiveTexture(GL_TEXTURE0 + i); // 在绑定之前激活相应的纹理单元
											  // 获取纹理序号（diffuse_textureN 中的 N）
			string number;
			string name = textures[i].type;
			if (name == "texture_diffuse")
				number = std::to_string(diffuseNr++);
			else if (name == "texture_specular")
				number = std::to_string(specularNr++);

			//shader->setInt((name + number).c_str(), i);
			glUniform1i(glGetUniformLocation(shader->ID, (name + number).c_str()), i);
			glBindTexture(GL_TEXTURE_2D, textures[i].id);
		}
		glActiveTexture(GL_TEXTURE0);

	} else {
		shader->setBool("useTexture", false);
		shader->setVec3("material.diffuse", this->material.diffuse);
		shader->setVec3("material.specular", this->material.specular);
		shader->setFloat("material.shininess", this->material.shininess);
	}
		
	// draw mesh
	glBindVertexArray(VAO);
	glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);

	glBindVertexArray(0);
}

Bounds3f CMesh::getBounds() { return this->bounds; }
//glm::mat4 getModel() { return this->model; }
glm::vec3 CMesh::getCentroid() { return this->bounds.getCentroid(); }

void CMesh::setMesh(vector<Vertex> vertices, vector<unsigned int> indices, MeshMaterial material) {
	this->vertices = vertices;
	this->indices = indices;
	this->textures = textures;
	this->material = material;

	// now that we have all the required data, set the vertex buffers and its attribute pointers.
	setupMesh();

	//deal with bounds
	float minx = 99999999.0f, miny = 99999999.0f, minz = 99999999.0f;
	float maxx = -99999999.0f, maxy = -99999999.0f, maxz = -99999999.0f;
	int countTemp = vertices.size();
	for (int i = 0; i < countTemp; ++i) {
		minx = vertices[i].Position.x < minx ? vertices[i].Position.x : minx;
		miny = vertices[i].Position.y < miny ? vertices[i].Position.y : miny;
		minz = vertices[i].Position.z < minz ? vertices[i].Position.z : minz;
		maxx = vertices[i].Position.x > maxx ? vertices[i].Position.x : maxx;
		maxy = vertices[i].Position.y > maxy ? vertices[i].Position.y : maxy;
		maxz = vertices[i].Position.z > maxz ? vertices[i].Position.z : maxz;
	}
	this->bounds = Bounds3f(glm::vec3(minx, miny, minz), glm::vec3(maxx, maxy, maxz));
}

void CMesh::setupMesh() {
	// create buffers/arrays
	glGenVertexArrays(1, &VAO);
	glGenBuffers(1, &VBO);
	glGenBuffers(1, &EBO);

	glBindVertexArray(VAO);
	// load data into vertex buffers
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	// A great thing about structs is that their memory layout is sequential for all its items.
	// The effect is that we can simply pass a pointer to the struct and it translates perfectly to a glm::vec3/2 array which
	// again translates to 3/2 floats which translates to a byte array.
	glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), &vertices[0], GL_STATIC_DRAW);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), &indices[0], GL_STATIC_DRAW);

	// set the vertex attribute pointers
	// vertex Positions
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)0);
	// vertex normals
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, Normal));

	glEnableVertexAttribArray(2);
	glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, TexCoords));

	glBindVertexArray(0);
}

GLuint CMesh::boundIndex[36] = {
	0, 1, 2, 0, 2, 3, 0, 5, 1, 0, 6, 5, 0, 3, 7, 0, 7, 6, 4, 5, 6, 4, 6, 7, 4, 2, 3, 4, 3, 7, 4, 5, 1, 4, 1, 2
};