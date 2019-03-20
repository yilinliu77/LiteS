#include "CPass.h"
#include <GL/glew.h>

CPass::CPass() {
	m_IsTargetTexture = false; 
}

CPass::~CPass() {}

void CPass::beginPass() {
	glViewport(0, 0, this->m_Width, this->m_Height);
	if (this->m_IsTargetTexture)
		glBindFramebuffer(GL_FRAMEBUFFER, this->m_FrameBuffer);

	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);


	this->getShader()->use();

	//this->m_Component->updateUniforms();
}

void CPass::endPass(CScene * vScene) {
	CShader* Shader = this->getShader();
	for (std::pair<std::string,CModel*> model: vScene->m_Models)
		model.second->draw(Shader);
	
	glUseProgram(0);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void CPass::endPass(CScene * vScene,vector<glm::mat4>& vModelMatrix) {
	CShader* Shader = this->getShader();
	for (std::pair<std::string, CModel*> model : vScene->m_Models)
		model.second->draw(Shader);

	glUseProgram(0);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}