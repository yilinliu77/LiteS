#pragma once

#ifndef CENGINE_H
#define CENGINE_H
#include <mutex>
#include "CComponent.h"
#include "CPass.h"
#include "CScene.h"
#include <GLFW/glfw3.h>

using namespace std;

enum RenderMode { debug, release };

class CEngine {
 public:
  // Solve mouse move
  static bool m_firstMouse;
  static float m_lastX;
  static float m_lastY;
  static bool m_mouseClicked;
  // timing
  static float m_deltaTime;
  static float m_lastFrame;

  static CScene* m_Scene;

  static map<string, CPass*> m_Pass;

  // Post Process
  static std::mutex m_addMeshMutex;
  static std::vector<std::pair<std::string, CModel*>> CEngine::toAddModels;

  CComponent* m_Component;

  std::thread* extraAlgorithm;

  static std::map<string, string> m_Arguments;

  CEngine();
  ~CEngine();
  bool initEngine(string configFile);
  void renderingLoop();
  void runEngine();
  bool isClicked(GLFWwindow* window, unsigned key);
  void handleInput(GLFWwindow* window);
  void switchMode(RenderMode v_mode);

 private:
  GLFWwindow* m_Window;

  bool __initDLL();
  void __generateAxis();
  bool __readProperties(string configFile);
  bool __initScene(string configFile);
  RenderMode m_mode;

  std::vector<bool> keyPreesed;

  GLfloat pointSize;

  bool show_demo_window = true;
};

#endif