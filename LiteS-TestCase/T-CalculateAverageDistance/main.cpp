#include "CEngine.h"
#include "CCalculateAverageDistanceComponent.h"

int main() {
  CEngine engine;
  if (!engine.initEngine("../../../LiteS-TestCase/T-CalculateAverageDistance/config.json",
                         "../../../LiteS-Engine/resources/")) {
    // if
    // (!engine.initEngine("C:/repos/GRAPHICS/RENDERING/LiteS/LiteS-TestCase/T-PathGenerate/config.json"))
    // {
    getchar();
    return 0;
  }
  engine.m_Component = new CCalculateAverageComponent(
      engine.m_Pass, engine.m_Scene,
                                                  CEngine::m_ResourceDir);
  // engine.switchMode(debug);

  engine.runEngine();

  return 0;
}
