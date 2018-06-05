#pragma once
#include <CComponent.h>
class CTexComponent : public CComponent {
public:
	CTexComponent(CScene * vScene);
	~CTexComponent();
	void run() override;
};

