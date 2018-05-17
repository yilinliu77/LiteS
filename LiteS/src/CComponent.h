#pragma once
#ifndef CCOMPONENT_H
#define CCOMPONENT_H
#include"CScene.h"
#include"CPass.h"
class CComponent {
public:
	CScene * m_Scene;

	CComponent(CScene * vScene) { m_Scene = vScene; }
	~CComponent() {};
	virtual void run() = 0;
	//virtual void updateUniforms() = 0;
	virtual bool extraInit();
};
#endif
