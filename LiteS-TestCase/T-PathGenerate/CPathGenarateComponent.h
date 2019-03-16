#pragma once
#include "CComponent.h"
#include <mutex>
#include "CPointCloudComponent.h"

class CPathGenarateComponent : public CPointCloudComponent {
public:
	CPathGenarateComponent(CScene * vScene);
	~CPathGenarateComponent();
	void generate_nadir();
	void optimize_nadir();
	void simplexPoint();
	void extraAlgorithm() override;
	void extraInit() override;

private:
};

