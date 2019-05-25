#ifndef LITE_UTIL_H
#define LITE_UTIL_H
#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>
#include<glm/gtc/random.hpp>
#include<glm/ext.hpp>

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/Exporter.hpp>
#include <assimp/scene.h>
#include<assimp/config.h>

#include <vector>
#include <array>
#include <numeric>
#include <algorithm>

#include "CMesh.h"
#include "CBVHACCEL.h"

using namespace Assimp;

struct My8BitRGBImage
{
	int ncols;
	int nrows;
	float* data;

};

float triangleArea(glm::vec3 v1, glm::vec3 v2, glm::vec3 v3);

void saveMesh(const CMesh* v_mesh, string v_outName);

void initializeSimplexes(float scale, glm::vec3 vPosition,
                         std::vector<glm::vec3>& solution,
                         size_t randomGenerator);

bool strongVisible(glm::vec3 vCameraPos, glm::vec3 vCameraOrientation,
                   glm::vec3 vSamplePosition, BVHAccel* vBVH, float vDMAX);

void shrink(std::vector<glm::vec3>* vSolution, glm::vec3 vPosition);

template<typename T>
std::vector<T> splitString(std::string str, std::string tok) {
	size_t splitPos;
	size_t psplitPos;
	istringstream ss;
	std::vector<T> out;
	T temp;
	splitPos = str.find(tok, 0);
	psplitPos = 0;
	while (splitPos!=std::string::npos)
	{
		ss.str(str.substr(psplitPos, splitPos));
		ss >> temp;
		out.push_back(temp);

		ss.str("");
		psplitPos = splitPos + 1;
		splitPos = str.find(tok, splitPos + 1);
	}
	ss.str(str.substr(psplitPos));
	ss >> temp;
	out.push_back(temp);
	return out;
}

#endif
