#pragma once
#include<CComponent.h>

class CSSAOComponent :public CComponent {
public:
	CSSAOComponent(CScene * vScene);
	~CSSAOComponent();

	void run() override;
	bool extraInit();

private:
	std::vector<glm::vec3> __SSAOKernel;
	std::vector<glm::vec3> __SSAONoise;

	void __initSampleKernel();

	float m_Near = 0.1f;
	float m_Far = 1000.0f;

};
