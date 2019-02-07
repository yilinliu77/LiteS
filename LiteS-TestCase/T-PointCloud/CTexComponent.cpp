#include "CTexComponent.h"

CTexComponent::CTexComponent(CScene * vScene) :CComponent(vScene){ 
	DisplayPass = this->m_Scene->m_Pass.at("display");
}

CTexComponent::~CTexComponent() = default;

void CTexComponent::run() {
	 
	DisplayPass->beginPass();

	const glm::mat4 projectionMatrix = glm::perspective(glm::radians(this->m_Scene->m_Camera->Zoom),
		static_cast<float>(this->m_Scene->m_WindowWidth) / static_cast<float>(this->m_Scene->m_WindowHeight),
												  0.1f, 1000000.0f);
	const glm::mat4 viewMatrix = this->m_Scene->m_Camera->GetViewMatrix();
	const glm::mat4 modelMatrix = glm::scale(glm::mat4(1.0f), glm::vec3(0.1, 0.1, 0.1));

	DisplayPass->getShader()->setMat4("projection", projectionMatrix);
	DisplayPass->getShader()->setMat4("model", modelMatrix);
	DisplayPass->getShader()->setMat4("view", viewMatrix);

	DisplayPass->getShader()->setVec3("color", glm::vec3(0.0f,0.0f,0.0f));

	DisplayPass->endPass();
}

void CTexComponent::extraAlgorithm() {

	for(int i=0;i<5;i++) {
		std::cout << i << std::endl;
		this->waitForContinueSignal();
	}
	std::cout << "init" << std::endl;

}

bool CTexComponent::extraInit() {

	return true;
}


