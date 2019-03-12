#pragma once
#ifndef CSCENE_H
#define CSCENE_H

#include"CModel.h"
#include"CCamera.h"
#include"CPass.h"
#include<map>

using namespace std;

class CScene {
public:
	CScene();
	~CScene();
	int m_WindowWidth;
	int m_WindowHeight;

	CCamera *m_Camera;

	map<string, GLuint> m_Texture;
	map<string, CPass*> m_Pass;
};

#endif