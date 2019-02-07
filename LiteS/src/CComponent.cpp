#include "CComponent.h"

CComponent::CComponent(CScene * vScene) :m_Scene(vScene)
	, m_shouldStep(false)
	, m_shouldContinue(false) {  }