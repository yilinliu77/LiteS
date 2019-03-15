#include "CEngine.h"
#include "CBVHDisplay.h"

int main(){
	CEngine engine;
	if (!engine.initEngine("C:/Users/vcc/Documents/repo/RENDERING/LiteS/LiteS-TestCase/T-BVHDisplay/config.xml")) {
	//if (!engine.initEngine("C:/repos/GRAPHICS/RENDERING/LiteS/LiteS-TestCase/T-PathPlaning/config.xml")) {
		getchar();
		return 0;
	}
	engine.m_Component = new CPathGenarateComponent(engine.m_Scene);
	//engine.switchMode(debug);

	engine.runEngine();

    return 0;
}
