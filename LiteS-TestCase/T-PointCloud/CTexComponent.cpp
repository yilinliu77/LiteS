#include "CTexComponent.h"
#include "../../LiteS/src/CPointCloudMesh.h"

CTexComponent::CTexComponent(CScene * vScene) :CPointCloudComponent(vScene){ 
	DisplayPass = this->m_Scene->m_Pass.at("display");
}

CTexComponent::~CTexComponent() = default;

void CTexComponent::extraAlgorithm() {
	CMesh* mesh = DisplayPass->m_Models[0]->meshes[0];

	for(int i=0;i<200;i++) {
		mesh->changeColor(glm::vec3(1.0f, 0.f, 0.f), i);
		cout << "change " << i << " to red" << endl;
		this->waitForStepSignal();
	}
	this->waitForContinueSignal();

	for (int i = 0; i < 5000; i++) {
		mesh->changeColor(glm::vec3(.0f, 1.f, 0.f), i);
		cout << "change " << i << " to yellow" << endl;
		this->waitForStepSignal();
	}
	this->waitForContinueSignal();

}

void CTexComponent::extraInit() {}


