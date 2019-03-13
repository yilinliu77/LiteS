#pragma once
#include "CComponent.h"
#include <mutex>
#include "CPointCloudComponent.h"

class CTexComponent : public CPointCloudComponent {
public:
	CTexComponent(CScene * vScene);
	~CTexComponent();
	void extraAlgorithm() override;
	void extraInit() override;

private:
	;
};

