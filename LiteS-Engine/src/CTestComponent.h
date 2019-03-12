#pragma once
#include "CComponent.h"
class CTestComponent :
	public CComponent {
public:
	CTestComponent(CScene * vScene);
	~CTestComponent();
	void run() override;
};