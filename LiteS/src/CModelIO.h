#ifndef MODELIO_HEADER
#define MODELIO_HEADER
#include <string>
#include <iostream>
#include "tinyply.h"
#include <fstream>

std::tuple<std::vector<Eigen::Vector3f>
, std::vector<Eigen::Vector3f>
, std::vector<int>
, std::vector<int>> loadPLY(std::string v_Filename,bool v_isBinary){
	std::cout << "Starting Reading the ply file" << std::endl;
	std::ifstream ss(v_Filename, v_isBinary?std::ios::binary:NULL);
	if (ss.fail())
		throw "failed to open ply file";

	tinyply::PlyFile file;
	file.parse_header(ss);

	std::shared_ptr<tinyply::PlyData> vertices, normals, faces, texcoords;
	try { vertices = file.request_properties_from_element("vertex", { "x", "y", "z" }); }
	catch (const std::exception & e) { std::cerr << "tinyply exception: " << e.what() << std::endl; }

	try { normals = file.request_properties_from_element("vertex", { "nx", "ny", "nz" }); }
	catch (const std::exception & e) { std::cerr << "tinyply exception: " << e.what() << std::endl; }

	try { texcoords = file.request_properties_from_element("vertex", { "u", "v" }); }
	catch (const std::exception & e) { std::cerr << "tinyply exception: " << e.what() << std::endl; }

	try { faces = file.request_properties_from_element("face", { "vertex_indices" }, 3); }
	catch (const std::exception & e) { std::cerr << "tinyply exception: " << e.what() << std::endl; }

	file.read(ss);

	std::vector<Eigen::Vector3f> vertexVector, normalVector;
	std::vector<int> texVector, faceVector;
	if (vertices) {
		std::cout << "\tRead " << vertices->count << " total vertices " << std::endl;
		const size_t numVerticesBytes = vertices->buffer.size_bytes();
		vertexVector= std::vector<Eigen::Vector3f>(vertices->count * 3);
		std::memcpy(vertexVector.data(), vertices->buffer.get(), numVerticesBytes);
	}
	if (normals) {
		std::cout << "\tRead " << normals->count << " total vertex normals " << std::endl;
		const size_t numNormalBytes = normals->buffer.size_bytes();
		normalVector= std::vector<Eigen::Vector3f>(normals->count * 3);
		std::memcpy(normalVector.data(), normals->buffer.get(), numNormalBytes);
	}
	if (texcoords) {
		std::cout << "\tRead " << texcoords->count << " total vertex texcoords " << std::endl;
		const size_t numTexBytes = texcoords->buffer.size_bytes();
		texVector= std::vector<int>(texcoords->count * 3);
		std::memcpy(texVector.data(), texcoords->buffer.get(), numTexBytes);
	}
	if (faces) {
		std::cout << "\tRead " << faces->count << " total faces (triangles) " << std::endl;
		const size_t numFacesBytes = faces->buffer.size_bytes();
		faceVector= std::vector<int>(faces->count * 3);
		std::memcpy(faceVector.data(), faces->buffer.get(), numFacesBytes);
	}

	ss.close();

	return std::make_tuple(vertexVector, normalVector, texVector, faceVector);
}

#endif // MODELIO_HEADER
