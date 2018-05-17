#include "CSSAOComponent.h"
#include <random>
#include <CMesh.h>

float gCubeVertices[] = {
	// back face
	-1.0f, -1.0f, -1.0f,  0.0f,  0.0f, -1.0f, 0.0f, 0.0f, // bottom-left
	1.0f,  1.0f, -1.0f,  0.0f,  0.0f, -1.0f, 1.0f, 1.0f, // top-right
	1.0f, -1.0f, -1.0f,  0.0f,  0.0f, -1.0f, 1.0f, 0.0f, // bottom-right         
	1.0f,  1.0f, -1.0f,  0.0f,  0.0f, -1.0f, 1.0f, 1.0f, // top-right
	-1.0f, -1.0f, -1.0f,  0.0f,  0.0f, -1.0f, 0.0f, 0.0f, // bottom-left
	-1.0f,  1.0f, -1.0f,  0.0f,  0.0f, -1.0f, 0.0f, 1.0f, // top-left
	// front face
	-1.0f, -1.0f,  1.0f,  0.0f,  0.0f,  1.0f, 0.0f, 0.0f, // bottom-left
	1.0f, -1.0f,  1.0f,  0.0f,  0.0f,  1.0f, 1.0f, 0.0f, // bottom-right
	1.0f,  1.0f,  1.0f,  0.0f,  0.0f,  1.0f, 1.0f, 1.0f, // top-right
	1.0f,  1.0f,  1.0f,  0.0f,  0.0f,  1.0f, 1.0f, 1.0f, // top-right
	-1.0f,  1.0f,  1.0f,  0.0f,  0.0f,  1.0f, 0.0f, 1.0f, // top-left
	-1.0f, -1.0f,  1.0f,  0.0f,  0.0f,  1.0f, 0.0f, 0.0f, // bottom-left
	// left face
	-1.0f,  1.0f,  1.0f, -1.0f,  0.0f,  0.0f, 1.0f, 0.0f, // top-right
	-1.0f,  1.0f, -1.0f, -1.0f,  0.0f,  0.0f, 1.0f, 1.0f, // top-left
	-1.0f, -1.0f, -1.0f, -1.0f,  0.0f,  0.0f, 0.0f, 1.0f, // bottom-left
	-1.0f, -1.0f, -1.0f, -1.0f,  0.0f,  0.0f, 0.0f, 1.0f, // bottom-left
	-1.0f, -1.0f,  1.0f, -1.0f,  0.0f,  0.0f, 0.0f, 0.0f, // bottom-right
	-1.0f,  1.0f,  1.0f, -1.0f,  0.0f,  0.0f, 1.0f, 0.0f, // top-right
	// right face
	1.0f,  1.0f,  1.0f,  1.0f,  0.0f,  0.0f, 1.0f, 0.0f, // top-left
	1.0f, -1.0f, -1.0f,  1.0f,  0.0f,  0.0f, 0.0f, 1.0f, // bottom-right
	1.0f,  1.0f, -1.0f,  1.0f,  0.0f,  0.0f, 1.0f, 1.0f, // top-right         
	1.0f, -1.0f, -1.0f,  1.0f,  0.0f,  0.0f, 0.0f, 1.0f, // bottom-right
	1.0f,  1.0f,  1.0f,  1.0f,  0.0f,  0.0f, 1.0f, 0.0f, // top-left
	1.0f, -1.0f,  1.0f,  1.0f,  0.0f,  0.0f, 0.0f, 0.0f, // bottom-left     
	// bottom face
	-1.0f, -1.0f, -1.0f,  0.0f, -1.0f,  0.0f, 0.0f, 1.0f, // top-right
	1.0f, -1.0f, -1.0f,  0.0f, -1.0f,  0.0f, 1.0f, 1.0f, // top-left
	1.0f, -1.0f,  1.0f,  0.0f, -1.0f,  0.0f, 1.0f, 0.0f, // bottom-left
	1.0f, -1.0f,  1.0f,  0.0f, -1.0f,  0.0f, 1.0f, 0.0f, // bottom-left
	-1.0f, -1.0f,  1.0f,  0.0f, -1.0f,  0.0f, 0.0f, 0.0f, // bottom-right
	-1.0f, -1.0f, -1.0f,  0.0f, -1.0f,  0.0f, 0.0f, 1.0f, // top-right
	// top face
	-1.0f,  1.0f, -1.0f,  0.0f,  1.0f,  0.0f, 0.0f, 1.0f, // top-left
	1.0f,  1.0f , 1.0f,  0.0f,  1.0f,  0.0f, 1.0f, 0.0f, // bottom-right
	1.0f,  1.0f, -1.0f,  0.0f,  1.0f,  0.0f, 1.0f, 1.0f, // top-right     
	1.0f,  1.0f,  1.0f,  0.0f,  1.0f,  0.0f, 1.0f, 0.0f, // bottom-right
	-1.0f,  1.0f, -1.0f,  0.0f,  1.0f,  0.0f, 0.0f, 1.0f, // top-left
	-1.0f,  1.0f,  1.0f,  0.0f,  1.0f,  0.0f, 0.0f, 0.0f  // bottom-left        
};

CSSAOComponent::CSSAOComponent(CScene * vScene) :CComponent(vScene) {}

CSSAOComponent::~CSSAOComponent() {}

void CSSAOComponent::run() {

	//1.Geometry Pass
	CPass* GeometryPass = this->m_Scene->m_Pass.at("geometry");
	GeometryPass->beginPass();
	glm::mat4 projectionMatrix = glm::perspective(this->m_Scene->m_Camera->Zoom,
		(float)this->m_Scene->m_WindowWidth / (float)this->m_Scene->m_WindowHeight,
												  m_Near, m_Far);
	glm::mat4 viewMatrix = this->m_Scene->m_Camera->GetViewMatrix();
	glm::mat4 modelMatrix = glm::scale(glm::mat4(1.0f), glm::vec3(0.1, 0.1, 0.1));

	GeometryPass->getShader()->setMat4("projection", projectionMatrix);
	GeometryPass->getShader()->setMat4("model", modelMatrix);
	GeometryPass->getShader()->setMat4("view", viewMatrix);

	GeometryPass->getShader()->setVec2("gProjParams", glm::vec2(projectionMatrix[2][2], projectionMatrix[3][2]));


	GeometryPass->endPass();

	//2.SSAO Pass
	CPass* SSAOPass = this->m_Scene->m_Pass.at("ssao");
	SSAOPass->beginPass();

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, this->m_Scene->m_Texture.at("position"));
	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, this->m_Scene->m_Texture.at("normal"));
	glActiveTexture(GL_TEXTURE2);
	glBindTexture(GL_TEXTURE_2D, this->m_Scene->m_Texture.at("ssaoNoise"));
	SSAOPass->getShader()->setInt("gPositionDepth", 0);
	SSAOPass->getShader()->setInt("gNormal", 1);
	SSAOPass->getShader()->setInt("texNoise", 2);
	for (GLuint i = 0; i < 64; ++i)
		SSAOPass->getShader()->setVec3("samples[" + to_string(i) + "]", this->__SSAOKernel[i]);

	SSAOPass->getShader()->setMat4("projection", projectionMatrix);

	SSAOPass->endPass();
	glBindTexture(GL_TEXTURE_2D, 0);

	//3.Blur Pass
	CPass* BlurPass = this->m_Scene->m_Pass.at("blur");
	BlurPass->beginPass();

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, this->m_Scene->m_Texture.at("ssaoColorBuffer"));
	BlurPass->getShader()->setInt("ssaoInput", 0);

	BlurPass->endPass();
	glBindTexture(GL_TEXTURE_2D, 0);

	//4.Shading Pass
	CPass* ShadingPass = this->m_Scene->m_Pass.at("shading");
	ShadingPass->beginPass();

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, this->m_Scene->m_Texture.at("gAlbedo"));
	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, this->m_Scene->m_Texture.at("ssaoColorBufferBlur"));

	ShadingPass->getShader()->setInt("gAlbedo", 0);
	ShadingPass->getShader()->setInt("ssao", 1);

	ShadingPass->endPass();
	glBindTexture(GL_TEXTURE_2D, 0);
}

bool CSSAOComponent::extraInit() {
	//noise texture
	__initSampleKernel();
	glBindTexture(GL_TEXTURE_2D, this->m_Scene->m_Texture.at("ssaoNoise"));
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F, 4, 4, 0, GL_RGB, GL_FLOAT, &__SSAONoise[0]);
	glBindTexture(GL_TEXTURE_2D, 0);

	//position texture
	//glBindTexture(GL_TEXTURE_2D, this->m_Scene->m_Texture.at("position"));
	//glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F, this->m_Scene->m_WindowWidth, this->m_Scene->m_WindowHeight, 0, GL_RGB, GL_FLOAT, 0);
	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	//glBindTexture(GL_TEXTURE_2D, 0);

	//Room
	//CPass* GeometryPass = this->m_Scene->m_Pass.at("geometry");
	//vector<Vertex> Vertices;
	//vector<unsigned int> Indices;
	//vector<Texture> Textures;

	//for (int i = 0; i < 36; ++i) {
	//	Vertex VertexTemp;
	//	VertexTemp.Position = glm::vec3(gCubeVertices[8 * i], gCubeVertices[8 * i + 1], gCubeVertices[8 * i + 2]);
	//	VertexTemp.Normal = glm::vec3(-gCubeVertices[8 * i + 3], -gCubeVertices[8 * i + 4], -gCubeVertices[8 * i + 5]);
	//	VertexTemp.TexCoords = glm::vec2(gCubeVertices[8 * i + 6], gCubeVertices[8 * i + 7]);

	//	Vertices.push_back(VertexTemp);
	//	Indices.push_back(i);
	//}
	//
	//GeometryPass->m_Models.push_back(new CModel());
	//GeometryPass->m_Models[1]->meshes[0] = new CMesh(Vertices, Indices, MeshMaterial(), Textures);

	return true;
}

float lerp(float a, float b, float f) {
	return a + f * (b - a);
}

void CSSAOComponent::__initSampleKernel()
{
	// Sample kernel
	std::uniform_real_distribution<GLfloat> randomFloats(0.0, 1.0); // generates random floats between 0.0 and 1.0
	std::default_random_engine generator;
	
	for (GLuint i = 0; i < 64; ++i) {
		glm::vec3 sample(randomFloats(generator) * 2.0 - 1.0, randomFloats(generator) * 2.0 - 1.0, randomFloats(generator));
		sample = glm::normalize(sample);
		sample *= randomFloats(generator);
		GLfloat scale = GLfloat(i) / (float)64.0;

		// Scale samples s.t. they're more aligned to center of kernel
		scale = lerp(0.1f, 1.0f, scale * scale);
		sample *= scale;
		this->__SSAOKernel.push_back(sample);
	}

	// Noise texture
	
	for (GLuint i = 0; i < 16; i++) {
		glm::vec3 noise(randomFloats(generator) * 2.0 - 1.0, randomFloats(generator) * 2.0 - 1.0, 0.0f); // rotate around z-axis (in tangent space)
		__SSAONoise.push_back(noise);
	}



}