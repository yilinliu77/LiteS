#pragma once
#include <mutex>
#include "CPointCloudComponent.h"

class CPathGenarateComponent : public CPointCloudComponent {
public:
  CPathGenarateComponent(const map<string, CPass*>& vPass, CScene* vScene,
                         const std::string vResourceDir);
	~CPathGenarateComponent();
	void extraAlgorithm() override;
	void extraInit() override;

private:
};

