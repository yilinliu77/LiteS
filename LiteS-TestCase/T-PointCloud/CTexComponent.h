#pragma once
#include "../../LiteS/src/CComponent.h"

class CTexComponent : public CComponent {
public:
	CTexComponent(CScene * vScene);
	~CTexComponent();
	void run() override;
};

