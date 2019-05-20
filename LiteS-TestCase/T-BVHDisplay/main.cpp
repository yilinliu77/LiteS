#include "CEngine.h"
#include "CBVHDisplay.h"

int main(){
	CEngine engine;
	if (!engine.initEngine("../../../LiteS-TestCase/T-BVHDisplay/config.json"
		,"../../../LiteS-Engine/resources/")) {
		getchar();
		return 0;
	}
        engine.m_Component = new CBVHDisplay(engine.m_Pass
		, engine.m_Scene, CEngine::m_ResourceDir);

	engine.runEngine();

    return 0;
}
