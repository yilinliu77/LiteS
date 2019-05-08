#include "CEngine.h"
#include "CGenerateNadirComponent.h"

int main() {
  CEngine engine;
  if (!engine.initEngine(
          "../../../../LiteS-TestCase/P/GenerateNadir/config.json",
          "../../../../LiteS-Engine/resources/")) {
    getchar();
    return 0;
  }
  engine.m_Component = new CGenerateNadirComponent(
      engine.m_Pass, engine.m_Scene, CEngine::m_ResourceDir);
  // engine.switchMode(debug);

  engine.runEngine();

  return 0;
}
