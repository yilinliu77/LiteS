#ifndef CPOINTCLOUDCOMPONENT_H
#define CPOINTCLOUDCOMPONENT_H
#include <mutex>
#include "CComponent.h"
#include "CPass.h"

class CPointCloudComponent : public CComponent {
 public:
  CPointCloudComponent(const map<string, CPass*>& vPass, CScene* vScene,
                       const std::string vResourceDir);

  void extraAlgorithm() override = 0;
  void stepExtraAlgorithm() override;
  void continueExtraAlgorithm() override;
  void waitForContinueSignal() override;
  void waitForStepSignal() override;
  void run() override;

  bool __generateRenderNormalPass();

  bool isRenderNormal;

  std::mutex m_shouldStepMutex;
  std::condition_variable m_shouldStepCV;

  std::mutex m_shouldContinueMutex;
  std::condition_variable m_shouldContinueCV;

  CPass* DisplayPass;

  CPass* normalRenderPass;
};

#endif
