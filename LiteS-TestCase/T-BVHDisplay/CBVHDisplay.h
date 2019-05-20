#pragma once
#include "CComponent.h"
#include <mutex>
#include "CPointCloudComponent.h"

class CBVHDisplay : public CPointCloudComponent {
 public:
  CBVHDisplay(const map<string, CPass*>& vPass, CScene* vScene,
              const std::string vResourceDir);
  ~CBVHDisplay();
	void generate_nadir();
	void optimize_nadir();
	void extraAlgorithm() override;
	void extraInit() override;

private:
	;
};

