#include "CEngine.h"
#include "CPathComponent.h"

int main(){
	CEngine engine;
	if (!engine.initEngine("../../../LiteS-TestCase/T-ProxyGenerate/config.json")) {
	//if (!engine.initEngine("C:/repos/GRAPHICS/RENDERING/LiteS/LiteS-TestCase/T-PathPlaning/config.xml")) {
		getchar();
		return 0;
	}
	engine.m_Component = new CPathComponent(engine.m_Pass,engine.m_Scene);
	//engine.switchMode(debug);

	engine.runEngine();

    return 0;
}
