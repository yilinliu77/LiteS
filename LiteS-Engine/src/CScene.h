#pragma once
#ifndef CSCENE_H
#define CSCENE_H

#include"CCamera.h"
#include<map>
#include <string>
#include "CTriMesh.h"
#include "CPointCloudMesh.h"

class CScene {
public:
	CScene();
	~CScene();
	int m_WindowWidth;
	int m_WindowHeight;

	CCamera *m_Camera;

	std::map<std::string, GLuint> m_Texture;

	std::vector<CMesh*> m_SystemModel;
	
	std::map<std::string, CMesh*> m_Models;
};

#endif