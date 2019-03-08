#pragma once
#include "../../LiteS/src/CComponent.h"
#include <mutex>
#include "../../LiteS/src/CPointCloudComponent.h"

class CPathComponent : public CPointCloudComponent {
public:
	CPathComponent(CScene * vScene);
	~CPathComponent();
	void extraAlgorithm() override;
	void extraInit() override;

private:
	;
};

