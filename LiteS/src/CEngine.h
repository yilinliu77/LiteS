#pragma once

#ifndef CENGINE_H
#define CENGINE_H

#include"tinyxml2.h"
#include"CComponent.h"
#include"CScene.h"
#include <GLFW/glfw3.h>


using namespace std;
using namespace tinyxml2;

class CEngine {
public:
	//Solve mouse move
	static bool m_firstMouse;
	static float m_lastX;
	static float m_lastY;
	// timing
	static float m_deltaTime;
	static float m_lastFrame;

	static CScene* m_Scene;

	CComponent* m_Component;

	CEngine();
	~CEngine();
	bool initEngine();
	void runEngine();
	void handleInput(GLFWwindow* window);

private:
	GLFWwindow * m_Window;

	bool __initDLL();
	bool __readProperties();
	bool __initScene();
};

#endif