#pragma once
#include <mutex>
#include "CPointCloudComponent.h"

class CVisualizeAnimationComponent : public CPointCloudComponent {
 public:
  CVisualizeAnimationComponent(const map<string, CPass*>& vPass, CScene* vScene,
                         const std::string vResourceDir);
  ~CVisualizeAnimationComponent();
  void staticsForPath(float reconThresh = 15.f);
  void visualizeMyAsiaCamera(string vPath);
  void extraAlgorithm() override;
  void extraInit() override;

 private:
};
