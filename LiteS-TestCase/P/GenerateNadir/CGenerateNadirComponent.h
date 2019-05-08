#pragma once
#include "CComponent.h"
#include <mutex>
#include "CPointCloudComponent.h"
#include "CPointCloudMesh.h"

class CGenerateNadirComponent : public CPointCloudComponent {
public:
	CGenerateNadirComponent(const map<string, CPass*>& vPass, CScene * vScene, std::string vResourceDir);
	~CGenerateNadirComponent(){}
	void generate_nadir();
	void extraAlgorithm() override;
	void extraInit() override;

private:
	CMesh* proxyPoint;
};

