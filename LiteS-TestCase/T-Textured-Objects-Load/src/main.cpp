#include<iostream>
#include<CEngine.h>
#include"CTexComponent.h"
int main() {
	CEngine engine;
	if (!engine.initEngine()) {
		getchar();
		return 0;
	}
	engine.m_Component = new CTexComponent(engine.m_Scene);

	engine.runEngine();
}