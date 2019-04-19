#pragma once

#include <Eigen/core>
#include <Eigen/SVD>  
#include <Eigen/Dense>  
#include <random>

#include <glm/glm.hpp>

struct CameraPose {
	glm::mat3 K = glm::mat3(1.0f, 0.f, 0.f, 0.f, 1.0f, 0.f, 0.f, 0.f, 1.f);
	glm::vec3 T = glm::vec3(0.0f, 0.f, 0.f);
	glm::mat3 R = glm::mat3(1.0f, 0.f, 0.f, 0.f, 1.0f, 0.f, 0.f, 0.f, 1.f);
	bool valied = true;
};

struct Viewport {
	size_t ID;

	std::vector<cv::KeyPoint>* keyPoints;
	cv::Mat* descriptor;

	std::vector<int> trackIDs;

	CameraPose pose;

	CImage<float> originalImage;
	size_t width, height;
};

struct PairWiseCamera {
	size_t ID1;
	size_t ID2;

	std::vector<std::pair<glm::vec3, glm::vec3>> matchResult;
	glm::mat3 foundamentalMatrix;
};

struct Track {
	bool valied = true;
	glm::vec3 pos;
	std::vector<std::pair<size_t, size_t>> tracks;
};

struct Observation {
	glm::vec2 pos;
	int camera_id;
	int point_id;
};

double sampsonDistance(Eigen::Matrix<float, 3, 3> &vFoundamentalMatrix
	, std::pair<glm::vec2, glm::vec2> vCoordinates) {
	double p2_F_p1 = 0.0;
	p2_F_p1 += vCoordinates.second[0] * (vCoordinates.first[0] * vFoundamentalMatrix(0, 0)
		+ vCoordinates.first[1] * vFoundamentalMatrix(0, 1) + vFoundamentalMatrix(0, 2));
	p2_F_p1 += vCoordinates.second[1] * (vCoordinates.first[0] * vFoundamentalMatrix(1, 0)
		+ vCoordinates.first[1] * vFoundamentalMatrix(1, 1) + vFoundamentalMatrix(1, 2));
	p2_F_p1 += 1.0 * (vCoordinates.first[0] * vFoundamentalMatrix(2, 0)
		+ vCoordinates.first[1] * vFoundamentalMatrix(2, 1) + vFoundamentalMatrix(2, 2));
	p2_F_p1 *= p2_F_p1;

	double sum = 0.0;
	sum += pow(vCoordinates.first[0] * vFoundamentalMatrix(0, 0)
		+ vCoordinates.first[1] * vFoundamentalMatrix(0, 1) + vFoundamentalMatrix(0, 2), 2);
	sum += pow(vCoordinates.first[0] * vFoundamentalMatrix(1, 0)
		+ vCoordinates.first[1] * vFoundamentalMatrix(1, 1) + vFoundamentalMatrix(1, 2), 2);
	sum += pow(vCoordinates.second[0] * vFoundamentalMatrix(0, 0)
		+ vCoordinates.second[1] * vFoundamentalMatrix(1, 0) + vFoundamentalMatrix(2, 0), 2);
	sum += pow(vCoordinates.second[0] * vFoundamentalMatrix(0, 1)
		+ vCoordinates.second[1] * vFoundamentalMatrix(1, 1) + vFoundamentalMatrix(2, 1), 2);

	return p2_F_p1 / sum;
}

float symmetricTransferError(glm::mat3& vHomographyMatrix
	, std::pair<glm::vec3, glm::vec3> vCoordinates) {
	const glm::vec3 point1 = vCoordinates.first;
	const glm::vec3 point2 = vCoordinates.second;
	/*
	 * Computes the symmetric transfer error for a given match and homography
	 * matrix. The error is computed as [Sect 4.2.2, Hartley, Zisserman]:
	 *
	 *   e = d(x, (H^-1)x')^2 + d(x', Hx)^2
	 */
	glm::vec3 p1(point1[0], point1[1], 1.0);
	glm::vec3 p2(point2[0], point2[1], 1.0);

	glm::mat3 invH = glm::inverse(vHomographyMatrix);
	glm::vec3 result =  p2 * invH;
	result /= result[2];
	float error = glm::pow(glm::distance(p1, result),2);

	result = p1* vHomographyMatrix;
	result /= result[2];
	error += glm::pow(glm::distance(result, p2),2);

	error = 0.5 * error;

	return error;
}

Eigen::VectorXf estimate8Points(const PairWiseCamera& pairCameras) {
	assert(pairCameras.matchResult.size() >= 8);

	std::random_device rd;
	std::uniform_int_distribution<size_t> gen(0, pairCameras.matchResult.size() - 1);
	std::mt19937 myRand(rd());

	std::set<size_t> result;
	while (result.size() < 8)
		result.insert(gen(myRand));

	Eigen::Matrix<float, 3, 8> pset1, pset2;
	std::set<size_t>::const_iterator iter = result.begin();
	for (int i = 0; i < 8; ++i, iter++) {
		pset1(0, i) = pairCameras.matchResult[*iter].first.x;
		pset1(1, i) = pairCameras.matchResult[*iter].first.y;
		pset1(2, i) = 1.f;
		pset2(0, i) = pairCameras.matchResult[*iter].second.x;
		pset2(1, i) = pairCameras.matchResult[*iter].second.y;
		pset2(2, i) = 1.f;
	}
	Eigen::Matrix<float, 8, 9> A;
	for (int i = 0; i < 8; ++i) {
		Eigen::Vector3f p1 = pset1.col(i);
		Eigen::Vector3f p2 = pset2.col(i);
		A(i, 0) = p2(0) * p1(0);
		A(i, 1) = p2(0) * p1(1);
		A(i, 2) = p2(0) * 1.0f;
		A(i, 3) = p2(1) * p1(0);
		A(i, 4) = p2(1) * p1(1);
		A(i, 5) = p2(1) * 1.0f;
		A(i, 6) = 1.0f   * p1(0);
		A(i, 7) = 1.0f   * p1(1);
		A(i, 8) = 1.0f   * 1.0f;
	}

	Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeFullV | Eigen::ComputeFullU);
	Eigen::Matrix<float, 8, 8> U = svd.matrixU();
	Eigen::Matrix<float, 9, 9> V = svd.matrixV();

	Eigen::VectorXf solution = V.col(8);

	return solution;
}

Eigen::Matrix<float, 3, 3> enforceConstrains(Eigen::VectorXf vSolution) {
	Eigen::Matrix<float, 3, 3> fundamentalMatrix;
	for (size_t y = 0; y < 3; y++)
		for (size_t x = 0; x < 3; x++)
			fundamentalMatrix(y, x) = vSolution(y * 3 + x);
	Eigen::JacobiSVD<Eigen::MatrixXf> svd(fundamentalMatrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::Matrix3f U = svd.matrixU();
	Eigen::Matrix3f V = svd.matrixV();
	Eigen::Matrix3f S = svd.singularValues().asDiagonal();

	S(2, 2) = 0;

	Eigen::Matrix<float, 3, 3> outPut;
	outPut = U * S*V.transpose();
	return outPut;
}

void poseFromEssential(const glm::mat3 vEssensialMatrix
	, std::vector<CameraPose>& vPoses) {
	glm::mat3 W(0.0);
	W[0][1] = -1.0; W[1][0] = 1.0; W[2][2] = 1.0;
	glm::mat3 Wt(0.0);
	Wt[0][1] = 1.0; Wt[1][0] = -1.0; Wt[2][2] = 1.0;

	Eigen::Matrix<float, 3, 3> E;
	E << vEssensialMatrix[0][0], vEssensialMatrix[0][1], vEssensialMatrix[0][2]
		, vEssensialMatrix[1][0], vEssensialMatrix[1][1], vEssensialMatrix[1][2]
		, vEssensialMatrix[2][0], vEssensialMatrix[2][1], vEssensialMatrix[2][2];

	Eigen::Matrix<float, 3, 3> U, S, V;
	Eigen::JacobiSVD<Eigen::MatrixXf> svd(E, Eigen::ComputeFullU | Eigen::ComputeFullV);
	U = svd.matrixU();
	V = svd.matrixV();
	
	if (U.determinant() < 0.0)
		for (int i = 0; i < 3; ++i)
			U(i, 2) = -U(i, 2);
	if (V.determinant() < 0.0)
		for (int i = 0; i < 3; ++i)
			V(i, 2) = -V(i, 2);

	V.transposeInPlace();
	glm::mat3 uGlm, vGlm, sGlm;
	for (int y = 0; y < 3; ++y)
		for (int x = 0; x < 3; ++x) {
			uGlm[y][x] = U(y, x);
			vGlm[y][x] = V(y, x);
		}
	vPoses.clear();
	vPoses.resize(4);
	vPoses.at(0).R = uGlm * W * vGlm;
	cout << U << endl;
	cout <<eigenFromGLM<3, 3>(W)<< endl;
	cout << V  << endl;
	vPoses.at(1).R = vPoses.at(0).R;
	vPoses.at(2).R = uGlm * Wt * vGlm;
	vPoses.at(3).R = vPoses.at(2).R;
	vPoses.at(0).T = glm::transpose(uGlm)[2];
	vPoses.at(1).T = -vPoses.at(0).T;
	vPoses.at(2).T = vPoses.at(0).T;
	vPoses.at(3).T = -vPoses.at(0).T;

	if (abs(glm::determinant(vPoses.at(0).R) - 1) > 1e-3)
		throw std::runtime_error("Invalid rotation matrix");
}

glm::vec3 triangulateMatch(const std::pair<glm::vec2, glm::vec2> vPairPos
	, CameraPose vPose1, CameraPose vPose2) {
	/* The algorithm is described in HZ 12.2, page 312. */
	Eigen::Matrix<float, 3, 4> P1, P2;
	Eigen::Matrix<float, 3, 3> KR1 = eigenFromGLM<3, 3>(vPose1.K * vPose1.R);
	Eigen::Matrix<float, 3, 1> Kt1 = eigenFromGLM<3>(vPose1.K * vPose1.T);
	P1.block(0, 0, 3, 3) = KR1;
	P1.block(0, 3, 3, 1) = Kt1;
	Eigen::Matrix<float, 3, 3> KR2 = eigenFromGLM<3, 3>(vPose2.K * vPose2.R);
	Eigen::Matrix<float, 3, 1> Kt2 = eigenFromGLM<3>(vPose2.K * vPose2.T);
	P2.block(0, 0, 3, 3) = KR2;
	P2.block(0, 3, 3, 1) = Kt2;

	Eigen::Matrix<float, 4, 4> A;
	for (int i = 0; i < 4; ++i) {
		A(0, i) = vPairPos.first[0] * P1(2, i) - P1(0, i);
		A(1, i) = vPairPos.first[1] * P1(2, i) - P1(1, i);
		A(2, i) = vPairPos.second[0] * P2(2, i) - P2(0, i);
		A(3, i) = vPairPos.second[1] * P2(2, i) - P2(1, i);
	}


	Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::Matrix<float, 4, 4> V = svd.matrixV();
	Eigen::VectorXf x = V.col(3);
	return glm::vec3(x[0] / x[3], x[1] / x[3], x[2] / x[3]);
}

glm::vec3 triangulateTrack(std::vector<glm::vec2>& vPostion, std::vector<CameraPose>& vPoses) {
	if (vPostion.size() != vPoses.size() || vPostion.size() < 2)
		throw std::invalid_argument("Invalid number of positions/poses");

	Eigen::MatrixXf A(2 * vPoses.size(), 4);
	for (std::size_t i = 0; i < vPoses.size(); ++i) {
		CameraPose const& pose = vPoses[i];
		glm::vec2 p = vPostion[i];
		Eigen::Matrix<float, 3, 4> P1;
		Eigen::Matrix<float, 3, 3> KR1 = eigenFromGLM<3, 3>(pose.K * pose.R);
		Eigen::Matrix<float, 3, 1> Kt1 = eigenFromGLM<3>(pose.K * pose.T);
		P1.block(0, 0, 3, 3) = KR1;
		P1.block(0, 3, 3, 1) = Kt1;

		for (int j = 0; j < 4; ++j) {
			A(2 * i + 0, j) = p[0] * P1(2, j) - P1(0, j);
			A(2 * i + 1, j) = p[1] * P1(2, j) - P1(1, j);
		}
	}
	/* Compute SVD. */
	Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::Matrix<float, 4, 4> V = svd.matrixV();
	Eigen::VectorXf x = V.col(3);
	return glm::vec3(x[0] / x[3], x[1] / x[3], x[2] / x[3]);
}
