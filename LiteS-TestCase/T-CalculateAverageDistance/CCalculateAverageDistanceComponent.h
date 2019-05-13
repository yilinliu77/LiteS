#pragma once
#include <mutex>
#include "CPointCloudComponent.h"

class CCalculateAverageComponent : public CPointCloudComponent {
 public:
  CCalculateAverageComponent(const map<string, CPass*>& vPass, CScene* vScene,
                         const std::string vResourceDir);
  ~CCalculateAverageComponent();
	void extraAlgorithm() override;
	void extraInit() override;

private:
};

