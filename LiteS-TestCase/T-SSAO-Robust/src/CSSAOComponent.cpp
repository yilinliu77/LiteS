#include "CSSAOComponent.h"
#include <nvToolsExt.h>
#include <random>
#include <CMesh.h>

CSSAOComponent::CSSAOComponent(CScene * vScene) :CComponent(vScene) {}

CSSAOComponent::~CSSAOComponent() {}

void CSSAOComponent::run() {

	nvtxRangePushA("Geometry");
	//1.Geometry Pass
	CPass* GeometryPass = this->m_Scene->m_Pass.at("geometry");
	GeometryPass->beginPass();
	glm::mat4 projectionMatrix = glm::perspective(this->m_Scene->m_Camera->Zoom,
		(float)this->m_Scene->m_WindowWidth / (float)this->m_Scene->m_WindowHeight,
												  m_Near, m_Far);
	GeometryPass->getShader()->setVec2("gProjParams", glm::vec2(projectionMatrix[2][2], projectionMatrix[3][2]));
	//GeometryPass->getShader()->setVec2("gProjParams", glm::vec2(m_Near,m_Far));
	glm::mat4 viewMatrix = this->m_Scene->m_Camera->GetViewMatrix();
	glm::mat4 modelMatrix = glm::mat4(1.0f);
	modelMatrix = glm::scale(modelMatrix, glm::vec3(0.1f));

	GeometryPass->getShader()->setMat4("projection", projectionMatrix);
	GeometryPass->getShader()->setMat4("model", modelMatrix);
	GeometryPass->getShader()->setMat4("view", viewMatrix);

	GeometryPass->endPass();
	nvtxRangePop();

	//2.DownSample Pass
	nvtxRangePushA("DownSample");
	CPass* DownSamplePass = this->m_Scene->m_Pass.at("downSample");
	DownSamplePass->beginPass();

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, this->m_Scene->m_Texture.at("depth"));
	DownSamplePass->getShader()->setInt("tex", 0);

	DownSamplePass->endPass();
	nvtxRangePop();


	//3.SSAO Pass
	nvtxRangePushA("SSAO");
	CPass* SSAOPass = this->m_Scene->m_Pass.at("ssao");
	SSAOPass->beginPass();

	//texture
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, this->m_Scene->m_Texture.at("quaterDepth"));
	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, this->m_Scene->m_Texture.at("normal"));
	SSAOPass->getShader()->setInt("gPositionDepth", 0);
	SSAOPass->getShader()->setInt("gNormal", 1);


	//Uniform
	SSAOPass->getShader()->setFloat("gAspect", (float)this->m_Scene->m_WindowWidth / (float)this->m_Scene->m_WindowHeight);
	SSAOPass->getShader()->setFloat("gRadiusWorld", 1.0f);
	SSAOPass->getShader()->setFloat("gMaxRadiusScreen", 0.1f);
	SSAOPass->getShader()->setFloat("gContrast", 4.0f);
	
	glm::vec2 size;
	size.y = 2.0f * 1.0f * glm::tan(0.5f * 3.14f / 3.0f);
	size.x = (float)this->m_Scene->m_WindowWidth / (float)this->m_Scene->m_WindowHeight * size.y;
	SSAOPass->getShader()->setFloat("gNearPlaneHeightNormalized", m_Near*glm::tan(this->m_Scene->m_Camera->Zoom/2));

	SSAOPass->endPass();
	nvtxRangePop();


	//4.Blur Pass X
	nvtxRangePushA("BlurX");
	CPass* BlurPass = this->m_Scene->m_Pass.at("blurX");
	BlurPass->beginPass();

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, this->m_Scene->m_Texture.at("ssaoColorBuffer"));
	BlurPass->getShader()->setInt("ssaoInput", 0);
	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, this->m_Scene->m_Texture.at("quaterDepth"));
	BlurPass->getShader()->setInt("depthInput", 1);

	BlurPass->getShader()->setVec2("texelSize", glm::vec2(2.0f / this->m_Scene->m_WindowWidth, 0.0f));

	BlurPass->endPass();
	nvtxRangePop();


	//5.Blur Pass Y
	nvtxRangePushA("BlurY");
	BlurPass = this->m_Scene->m_Pass.at("blurY");
	BlurPass->beginPass();

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, this->m_Scene->m_Texture.at("ssaoColorBufferBlur"));
	BlurPass->getShader()->setInt("ssaoInput", 0);
	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, this->m_Scene->m_Texture.at("quaterDepth"));
	BlurPass->getShader()->setInt("depthInput", 1);

	BlurPass->getShader()->setVec2("texelSize", glm::vec2(0.0f, 2.0f / this->m_Scene->m_WindowHeight));

	BlurPass->endPass();
	nvtxRangePop();


	//6.UpSample Pass
	nvtxRangePushA("UpSample");
	CPass* UpSamplePass = this->m_Scene->m_Pass.at("upSample");
	UpSamplePass->beginPass();

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, this->m_Scene->m_Texture.at("depth"));
	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, this->m_Scene->m_Texture.at("quaterDepth"));
	glActiveTexture(GL_TEXTURE2);
	glBindTexture(GL_TEXTURE_2D, this->m_Scene->m_Texture.at("ssaoColorBuffer"));

	UpSamplePass->getShader()->setInt("depthTexture", 0);
	UpSamplePass->getShader()->setInt("quaterDepthTexture", 1);
	UpSamplePass->getShader()->setInt("ssaoTexture", 2);

	UpSamplePass->getShader()->setVec2("gProjParams", glm::vec2(projectionMatrix[2][2], projectionMatrix[3][2]));
	UpSamplePass->getShader()->setVec2("gPixelSize", glm::vec2(1.0f / this->m_Scene->m_WindowWidth, 1.0f / this->m_Scene->m_WindowHeight));

	UpSamplePass->endPass();
	nvtxRangePop();


	////Debug.SSAOShow pass
	//CPass* SSAOShow = this->m_Scene->m_Pass.at("showSSAO");
	//SSAOShow->beginPass();

	//glActiveTexture(GL_TEXTURE0);
	//glBindTexture(GL_TEXTURE_2D, this->m_Scene->m_Texture.at("upSampleSSAO"));

	//SSAOShow->getShader()->setInt("gSSAO", 0);

	//SSAOShow->endPass();

	//7.Shading Pass
	nvtxRangePushA("Shading");
	CPass* ShadingPass = this->m_Scene->m_Pass.at("shading");
	ShadingPass->beginPass();

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, this->m_Scene->m_Texture.at("depth"));
	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, this->m_Scene->m_Texture.at("normal"));
	glActiveTexture(GL_TEXTURE2);
	glBindTexture(GL_TEXTURE_2D, this->m_Scene->m_Texture.at("gAlbedo"));
	glActiveTexture(GL_TEXTURE3);
	glBindTexture(GL_TEXTURE_2D, this->m_Scene->m_Texture.at("ssaoColorBuffer"));

	ShadingPass->getShader()->setInt("gPositionDepth", 0);
	ShadingPass->getShader()->setInt("gNormal", 1);
	ShadingPass->getShader()->setInt("gAlbedo", 2);
	ShadingPass->getShader()->setInt("ssao", 3);

	ShadingPass->endPass();
	nvtxRangePop();


	frameCount++;
	//if (frameCount == 1000)
	//	exit(0);
}

bool CSSAOComponent::extraInit() {
	//noise texture
	//__initSampleKernel();
	//glBindTexture(GL_TEXTURE_2D, this->m_Scene->m_Texture.at("ssaoNoise"));
	//glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F, 4, 4, 0, GL_RGB, GL_FLOAT, &__SSAONoise[0]);
	//glBindTexture(GL_TEXTURE_2D, 0);

	//position texture
	/*glBindTexture(GL_TEXTURE_2D, this->m_Scene->m_Texture.at("position"));
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F, this->m_Scene->m_WindowWidth, this->m_Scene->m_WindowHeight, 0, GL_RGB, GL_FLOAT, 0);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glBindTexture(GL_TEXTURE_2D, 0);*/

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