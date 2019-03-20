#include "CComponent.h"

CComponent::CComponent(const map<string, CPass*>& vPass, CScene * vScene) : m_shouldStep(false)
	, m_shouldContinue(false),m_Pass(vPass),m_Scene(vScene) {  }