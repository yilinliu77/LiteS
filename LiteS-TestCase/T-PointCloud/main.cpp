#include "CEngine.h"
#include "CTexComponent.h"

int main(){
	CEngine engine;
	if (!engine.initEngine("C:/repos/Graphics/RENDERING/LiteS/LiteS-TestCase/T-PointCloud/config.xml")) {
		getchar();
		return 0;
	}
	engine.m_Component = new CTexComponent(engine.m_Scene);
	//engine.switchMode(debug);

	engine.runEngine();

    return 0;
}
