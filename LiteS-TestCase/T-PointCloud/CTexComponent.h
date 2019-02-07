#pragma once
#include "../../LiteS/src/CComponent.h"
#include <mutex>
#include "../../LiteS/src/CPointCloudComponent.h"

class CTexComponent : public CPointCloudComponent {
public:
	CTexComponent(CScene * vScene);
	~CTexComponent();
	void extraAlgorithm() override;
	void extraInit() override;

private:
	;
};

