#ifndef CDIFFERPASS_H
#define CDIFFERPASS_H

#include "CPass.h"

class CDifferPass:public CPass {
public:
	void beginPass();
	void endPass();
	void appendRender();
};
#endif