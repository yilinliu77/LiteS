#pragma once

#include <Eigen/core>
#include <Eigen/SVD>  
#include <Eigen/Dense>  
#include <random>

const float errorTriangulateThreshold = 0.05f;

struct CameraPose {
	Eigen::Matrix3f K = (Eigen::Matrix3f() << 1.0f, 0.f, 0.f, 0.f, 1.0f, 0.f, 0.f, 0.f, 1.f).finished();
	Eigen::Vector3f T = Eigen::Vector3f(0.0f, 0.f, 0.f);
	Eigen::Matrix3f R = (Eigen::Matrix3f()<<1.0f, 0.f, 0.f, 0.f, 1.0f, 0.f, 0.f, 0.f, 1.f).finished();
	bool valied = false;
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

	std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> matchResult;
	Eigen::Matrix3f foundamentalMatrix;
};

struct Track {
	bool valied = false;
	Eigen::Vector3f pos;
	std::vector<std::pair<size_t, size_t>> tracks;
};

struct Observation {
	Eigen::Vector2f pos;
	int camera_id;
	int point_id;
};

double sampsonDistance(Eigen::Matrix<float, 3, 3> &vFoundamentalMatrix
	,const std::pair<Eigen::Vector3f, Eigen::Vector3f>& vCoordinates) {
	double p2_F_p1 = 0.0;
	p2_F_p1 += vCoordinates.second(0) * (vCoordinates.first(0) * vFoundamentalMatrix(0, 0)
		+ vCoordinates.first(1) * vFoundamentalMatrix(0, 1) + vFoundamentalMatrix(0, 2));
	p2_F_p1 += vCoordinates.second(1) * (vCoordinates.first(0) * vFoundamentalMatrix(1, 0)
		+ vCoordinates.first(1) * vFoundamentalMatrix(1, 1) + vFoundamentalMatrix(1, 2));
	p2_F_p1 += 1.0 * (vCoordinates.first(0) * vFoundamentalMatrix(2, 0)
		+ vCoordinates.first(1) * vFoundamentalMatrix(2, 1) + vFoundamentalMatrix(2, 2));
	p2_F_p1 *= p2_F_p1;

	double sum = 0.0;
	sum += pow(vCoordinates.first(0) * vFoundamentalMatrix(0, 0)
		+ vCoordinates.first(1) * vFoundamentalMatrix(0, 1) + vFoundamentalMatrix(0, 2), 2);
	sum += pow(vCoordinates.first(0) * vFoundamentalMatrix(1, 0)
		+ vCoordinates.first(1) * vFoundamentalMatrix(1, 1) + vFoundamentalMatrix(1, 2), 2);
	sum += pow(vCoordinates.second(0) * vFoundamentalMatrix(0, 0)
		+ vCoordinates.second(1) * vFoundamentalMatrix(1, 0) + vFoundamentalMatrix(2, 0), 2);
	sum += pow(vCoordinates.second(0) * vFoundamentalMatrix(0, 1)
		+ vCoordinates.second(1) * vFoundamentalMatrix(1, 1) + vFoundamentalMatrix(2, 1), 2);

	return p2_F_p1 / sum;
}

float symmetricTransferError(Eigen::Matrix3f& vHomographyMatrix
	, std::pair<Eigen::Vector3f, Eigen::Vector3f> vCoordinates) {
	const Eigen::Vector3f& point1 = vCoordinates.first;
	const Eigen::Vector3f& point2 = vCoordinates.second;
	/*
	 * Computes the symmetric transfer error for a given match and homography
	 * matrix. The error is computed as [Sect 4.2.2, Hartley, Zisserman]:
	 *
	 *   e = d(x, (H^-1)x')^2 + d(x', Hx)^2
	 */
	Eigen::Vector3f p1(point1[0], point1[1], 1.0f);
	Eigen::Vector3f p2(point2[0], point2[1], 1.0f);

	Eigen::Matrix3f invH = vHomographyMatrix.inverse();
	Eigen::Vector3f result = invH * p2  ;
	result /= result[2];
	float error = std::pow((p1- result).norm(),2);

	result = vHomographyMatrix*p1 ;
	result /= result[2];
	error += std::pow((result-p2).norm(),2);

	error = 0.5f * error;

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
		pset1(0, i) = pairCameras.matchResult[*iter].first(0);
		pset1(1, i) = pairCameras.matchResult[*iter].first(1);
		pset1(2, i) = 1.f;
		pset2(0, i) = pairCameras.matchResult[*iter].second(0);
		pset2(1, i) = pairCameras.matchResult[*iter].second(1);
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

void poseFromEssential(const Eigen::Matrix3f vEssensialMatrix
	, std::vector<CameraPose>& vPoses) {
	Eigen::Matrix3f W=Eigen::Matrix3f::Zero(3,3);
	W(0,1) = -1.0; W(1,0) = 1.0; W(2,2) = 1.0;
	Eigen::Matrix3f Wt = Eigen::Matrix3f::Zero(3, 3);
	Wt(0, 1) = 1.0; Wt(1, 0) = -1.0; Wt(2, 2) = 1.0;

	Eigen::Matrix<float, 3, 3> U, S, V;
	Eigen::JacobiSVD<Eigen::MatrixXf> svd(vEssensialMatrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
	U = svd.matrixU();
	V = svd.matrixV();
	
	if (U.determinant() < 0.0)
		for (int i = 0; i < 3; ++i)
			U(i, 2) = -U(i, 2);
	if (V.determinant() < 0.0)
		for (int i = 0; i < 3; ++i)
			V(i, 2) = -V(i, 2);
	
	vPoses.clear();
	vPoses.resize(4);
	Eigen::Matrix<float, 3, 3> myRotation = U * W*V.transpose();
	vPoses.at(0).R = myRotation;
	vPoses.at(1).R = vPoses.at(0).R;
	vPoses.at(2).R = U*Wt*V.transpose();
	vPoses.at(3).R = vPoses.at(2).R;
	vPoses.at(0).T = U.col(2);
	vPoses.at(1).T = -vPoses.at(0).T;
	vPoses.at(2).T = vPoses.at(0).T;
	vPoses.at(3).T = -vPoses.at(0).T;

	//cout << eigenFromGLM<3, 3>(vPoses.at(0).R) << endl;
	//cout << eigenFromGLM<3, 3>(vPoses.at(2).R) << endl;
	//cout << eigenFromGLM<3>(vPoses.at(0).T) << endl;


	if (abs((vPoses.at(0).R).determinant() - 1) > 1e-3)
		throw std::runtime_error("Invalid rotation matrix");
}

Eigen::Vector3f triangulateMatch(const std::pair<Eigen::Vector2f, Eigen::Vector2f> vPairPos
	, CameraPose vPose1, CameraPose vPose2) {
	/* The algorithm is described in HZ 12.2, page 312. */
	Eigen::Matrix<float, 3, 4> P1, P2;
	Eigen::Matrix<float, 3, 3> KR1 = vPose1.K * vPose1.R;
	Eigen::Matrix<float, 3, 1> Kt1 = vPose1.K * vPose1.T;
	P1.block(0, 0, 3, 3) = KR1;
	P1.block(0, 3, 3, 1) = Kt1;
	Eigen::Matrix<float, 3, 3> KR2 = vPose2.K * vPose2.R;
	Eigen::Matrix<float, 3, 1> Kt2 = vPose2.K * vPose2.T;
	P2.block(0, 0, 3, 3) = KR2;
	P2.block(0, 3, 3, 1) = Kt2;

	Eigen::Matrix<float, 4, 4> A;
	for (int i = 0; i < 4; ++i) {
		A(0, i) = vPairPos.first(0) * P1(2, i) - P1(0, i);
		A(1, i) = vPairPos.first(1) * P1(2, i) - P1(1, i);
		A(2, i) = vPairPos.second(0) * P2(2, i) - P2(0, i);
		A(3, i) = vPairPos.second(1) * P2(2, i) - P2(1, i);
	}


	Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::Matrix<float, 4, 4> V = svd.matrixV();
	Eigen::VectorXf x = V.col(3);
	return Eigen::Vector3f(x(0) / x(3), x(1) / x(3), x(2) / x(3));
}

Eigen::Vector3f triangulateTrack(std::vector<Eigen::Vector2f>& vPostion
	, std::vector<CameraPose>& vPoses) {
	if (vPostion.size() != vPoses.size() || vPostion.size() < 2)
		throw std::invalid_argument("Invalid number of positions/poses");

	Eigen::MatrixXf A(2 * vPoses.size(), 4);
	for (std::size_t i = 0; i < vPoses.size(); ++i) {
		CameraPose const& pose = vPoses[i];
		Eigen::Vector2f p = vPostion[i];
		Eigen::Matrix<float, 3, 4> P1;
		Eigen::Matrix<float, 3, 3> KR1 = pose.K * pose.R;
		Eigen::Matrix<float, 3, 1> Kt1 = pose.K * pose.T;
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
	return Eigen::Vector3f(x(0) / x(3), x(1) / x(3), x(2) / x(3));
}
