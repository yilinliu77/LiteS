#pragma once
#include <mutex>
#include "CPointCloudComponent.h"

class CPathGenarateComponent : public CPointCloudComponent {
public:
	CPathGenarateComponent(const map<string, CPass*>& vPass,CScene * vScene);
	~CPathGenarateComponent();
	void visualizeCamera(size_t vCameraIndex);
	void generate_nadir();
	void updateRays(int vCameraIndex, glm::vec3 vNewPosition);
	glm::vec3 optimizeOrientation(glm::vec3 vCameraPosition, int vCameraIndex);
	std::pair<glm::vec3, float> downhillSimplex(std::vector<glm::vec3>* solution, size_t vCameraIndex, std::function<float(glm::vec3, glm::vec3, size_t)> const & func);
	void optimize(size_t vCameraIndex, int vIter);
	void optimize_nadir();
	void simplexPoint();
	void extraAlgorithm() override;
	void extraInit() override;

private:
};

