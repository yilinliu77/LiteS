#pragma once
#include <mutex>
#include "CPointCloudComponent.h"

class CEvaluatePointCloudComponent : public CPointCloudComponent {
public:
	CEvaluatePointCloudComponent(const map<string, CPass*>& vPass, CScene* vScene,
                         const std::string vResourceDir);
	~CEvaluatePointCloudComponent();
	void extraAlgorithm() override;
	void extraInit() override;

private:
};

