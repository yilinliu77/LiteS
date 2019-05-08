#include "CTestComponent.h"

CTestComponent::CTestComponent(const map<string, CPass*>& vPass, CScene* vScene,
                               const std::string vResourceDir)
    : CComponent(vPass, vScene,vResourceDir){};

CTestComponent::~CTestComponent() {}

void CTestComponent::run() {
  CPass* DisplayPass = this->m_Pass.at("display");
  DisplayPass->beginPass();

  glm::mat4 projectionMatrix =
      glm::perspective(glm::radians(this->m_Scene->m_Camera->Zoom),
                       (float)this->m_Scene->m_WindowWidth /
                           (float)this->m_Scene->m_WindowHeight,
                       0.1f, 1000.0f);
  glm::mat4 viewMatrix = this->m_Scene->m_Camera->GetViewMatrix();
  glm::mat4 modelMatrix =
      glm::scale(glm::mat4(1.0f), glm::vec3(0.2f, 0.2f, 0.2f));

  DisplayPass->getShader()->setMat4("projection", projectionMatrix);
  DisplayPass->getShader()->setMat4("model", modelMatrix);
  DisplayPass->getShader()->setMat4("view", viewMatrix);
  // DisplayPass->getShader()->setMat4("matrix",
  // projectionMatrix*viewMatrix*modelMatrix);
  DisplayPass->getShader()->setVec3("viewPos",
                                    this->m_Scene->m_Camera->Position);

  DisplayPass->endPass(this->m_Scene);
}