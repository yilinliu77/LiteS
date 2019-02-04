#pragma once
#ifndef CCOMPONENT_H
#define CCOMPONENT_H
#include"CScene.h"
#include"CPass.h"
#include <mutex>

class CComponent {
public:
	CScene * m_Scene;

	CComponent(CScene * vScene) :m_shouldContinue(false), m_Scene(vScene) { }
	~CComponent() {};
	virtual void run() = 0;
	//virtual void updateUniforms() = 0;
	virtual bool extraInit();
	virtual void extraAlgorithm();
	virtual void continueExtraInit();
	virtual void waitForContinueSignal();

	std::mutex m_shouldContinueMutex;
	std::condition_variable m_shouldContinueCV;
	bool m_shouldContinue;
};
#endif
