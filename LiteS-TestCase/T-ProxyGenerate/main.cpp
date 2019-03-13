#include "CEngine.h"
#include "CPathComponent.h"

int main(){
	CEngine engine;
	if (!engine.initEngine("C:/Users/vcc/Documents/repo/RENDERING/LiteS/LiteS-TestCase/T-ProxyGenerate/config.xml")) {
	//if (!engine.initEngine("C:/repos/GRAPHICS/RENDERING/LiteS/LiteS-TestCase/T-PathPlaning/config.xml")) {
		getchar();
		return 0;
	}
	engine.m_Component = new CPathComponent(engine.m_Scene);
	//engine.switchMode(debug);

	engine.runEngine();

    return 0;
}
