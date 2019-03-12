#pragma once
#ifndef CPASS_H
#define CPASS_H

#include"CShader.h"
#include"CModel.h"
#include <GL/glew.h>

using namespace std;

class CPass {
public:
	CPass();
	~CPass();

	CShader* getShader() { return m_Shader; }
	bool setShader(const char* vVertFile, const char* vFragFile) {
		m_Shader = new CShader(vVertFile, vFragFile);
		if (!m_Shader->checkShaderErrors())
			return false;
		return true;
	}

	void beginPass();
	void endPass();

	void endPass(vector<glm::mat4>& vModelMatrix);

	GLuint m_FrameBuffer;
	bool m_IsTargetTexture;
	vector<CModel*> m_Models;
	unsigned int m_Width;
	unsigned int m_Height;


private:
	CShader * m_Shader;
	vector<GLuint> m_Textures;
	vector<GLuint> m_TargetFrame;
	
};
#endif