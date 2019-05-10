#include "CEngine.h"
#include "CEvaluatePointCloud.h"

int main() {
  CEngine engine;
  if (!engine.initEngine("../../../LiteS-TestCase/T-RemovePointInside/config.json",
                         "../../../LiteS-Engine/resources/")) {
    // if
    // (!engine.initEngine("C:/repos/GRAPHICS/RENDERING/LiteS/LiteS-TestCase/T-PathGenerate/config.json"))
    // {
    getchar();
    return 0;
  }
  engine.m_Component = new CEvaluatePointCloudComponent(engine.m_Pass, engine.m_Scene,
                                                  CEngine::m_ResourceDir);
  // engine.switchMode(debug);

  engine.runEngine();

  return 0;
}
