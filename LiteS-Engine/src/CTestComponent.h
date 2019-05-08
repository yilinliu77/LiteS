#pragma once
#include "CComponent.h"
#include "CPass.h"
class CTestComponent : public CComponent {
 public:
  CTestComponent(const map<string, CPass*>& vPass, CScene* vScene,
                 const std::string vResourceDir);
  ~CTestComponent();
  void run() override;
};