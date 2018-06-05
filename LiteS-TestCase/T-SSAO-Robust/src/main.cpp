#include<iostream>
#include<CEngine.h>
#include"CSSAOComponent.h"
int main() {
	CEngine engine;
	if (!engine.initEngine()) {
		getchar();
		return 0;
	}
	engine.m_Component = new CSSAOComponent(engine.m_Scene);

	engine.runEngine();
}