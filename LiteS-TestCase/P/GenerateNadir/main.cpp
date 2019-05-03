#include "CEngine.h"
#include "CGenerateNadirComponent.h"

int main() {
	CEngine engine;
	if (!engine.initEngine("../../../../LiteS-TestCase/P/GenerateNadir/config.json")) {
		getchar();
		return 0;
	}
	engine.m_Component = new CGenerateNadirComponent(engine.m_Pass, engine.m_Scene);
	//engine.switchMode(debug);

	engine.runEngine();

	return 0;
}
