#pragma once
#ifndef CCOMPONENT_H
#define CCOMPONENT_H
#include"CScene.h"
#include "CPass.h"

class CComponent {
public:

	CComponent(const map<string, CPass*>& vPass,CScene * vScene,const std::string vResourceDir);

	virtual ~CComponent() = default;
	virtual void run() = 0;
	virtual void extraInit() = 0;
	
	virtual void extraAlgorithm() = 0;
	virtual void stepExtraAlgorithm() = 0;
	virtual void continueExtraAlgorithm() = 0;
	virtual void waitForStepSignal() = 0;
	virtual void waitForContinueSignal() = 0;

	bool m_shouldContinue;
	bool m_shouldStep;

	map<string, CPass*> m_Pass;
	CScene * m_Scene;
	std::string m_ResourceDir;
};
#endif
