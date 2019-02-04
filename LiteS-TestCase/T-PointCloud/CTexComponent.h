#pragma once
#include "../../LiteS/src/CComponent.h"
#include <mutex>

class CTexComponent : public CComponent {
public:
	CTexComponent(CScene * vScene);
	~CTexComponent();
	void run() override;
	void extraAlgorithm() override;

private:
	
};

