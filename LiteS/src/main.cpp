#include<iostream>
#include"CEngine.h"
#include"CTestComponent.h"
int main() {
	CEngine engine;
	if (!engine.initEngine()) {
		getchar();
		return 0;
	}
	engine.m_Component = new CTestComponent(engine.m_Scene);

	engine.runEngine();
}