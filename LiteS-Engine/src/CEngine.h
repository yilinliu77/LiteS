#pragma once

#ifndef CENGINE_H
#define CENGINE_H

#include"tinyxml2.h"
#include"CComponent.h"
#include"CScene.h"
#include <GLFW/glfw3.h>
#include <mutex>

using namespace std;
using namespace tinyxml2;

enum RenderMode {
	debug,release
};

class CEngine {
public:
	//Solve mouse move
	static bool m_firstMouse;
	static float m_lastX;
	static float m_lastY;
	static bool m_mouseClicked;
	// timing
	static float m_deltaTime;
	static float m_lastFrame;

	static CScene* m_Scene;

	//Post Process
	static std::mutex m_addMeshMutex;
	static std::vector<CMesh*> toAddMeshes;

	CComponent* m_Component;

	CEngine();
	~CEngine();
	bool initEngine(string configFile);
	void runEngine();
	bool isClicked(GLFWwindow* window, unsigned key);
	void handleInput(GLFWwindow* window);
	void switchMode(RenderMode v_mode);

private:
	GLFWwindow * m_Window;

	bool __initDLL();
	bool __readProperties(string configFile);
	bool __initScene(string configFile);
	RenderMode m_mode;

	std::vector<bool> keyPreesed;

	GLfloat pointSize;

	bool show_demo_window = true;
};

#endif