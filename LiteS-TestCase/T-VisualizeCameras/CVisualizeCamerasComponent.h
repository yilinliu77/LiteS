#pragma once
#include <mutex>
#include "CPointCloudComponent.h"

class CVisualizeCamerasComponent : public CPointCloudComponent {
 public:
  CVisualizeCamerasComponent(const map<string, CPass*>& vPass, CScene* vScene,
                         const std::string vResourceDir);
  ~CVisualizeCamerasComponent();
  void staticsForPath(float reconThresh = 15.f);
  void visualizeMyAsiaCamera(string vPath);
  void extraAlgorithm() override;
  void extraInit() override;

 private:
};
