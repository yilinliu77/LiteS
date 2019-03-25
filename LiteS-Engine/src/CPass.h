#pragma once
#ifndef CPASS_H
#define CPASS_H

#include"CShader.h"
#include"CModel.h"
#include "CScene.h"
#include <GL/glew.h>

using namespace std;

class CPass {
public:
	CPass(bool isNormal);
	~CPass();

	CShader* getShader() { return m_Shader; }
	bool setShader(const char* vVertFile, const char* vFragFile, const char* vGeomFile=NULL) {
		m_Shader = new CShader(vVertFile, vFragFile, vGeomFile);
		if (!m_Shader->checkShaderErrors())
			return false;
		return true;
	}

	void beginPass();
	void endPass(CScene * vScene);

	void endPass(CScene * vScene,vector<glm::mat4>& vModelMatrix);

	GLuint m_FrameBuffer;
	bool m_IsTargetTexture;
	bool m_IsNormal;
	
	unsigned int m_Width;
	unsigned int m_Height;


private:
	CShader * m_Shader;
	vector<GLuint> m_Textures;
	vector<GLuint> m_TargetFrame;
	
};
#endif