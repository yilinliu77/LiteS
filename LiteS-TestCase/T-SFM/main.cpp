#include "CSIFT.h"
#include "FreeImage.h"
#include <glm/matrix.hpp>
#include <Eigen/Core>
#include <Eigen/SVD>  
#include <Eigen/Dense>  
#include <iostream>
#include <random>
#include "CImage.h"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/xfeatures2d/nonfree.hpp>

const float RANSAC_INLIER_THRESHOLD = 0.0015f;

struct PairWiseCameras {
	CImage<float>* image1;
	CImage<float>* image2;
	
	std::vector<std::pair<glm::vec2, glm::vec2>> matchResult;
};

PairWiseCameras getMatch(const std::string vImagePath1, const std::string vImagePath2,bool display=true) {
	CSIFT* detector1 = new CSIFT();
	CSIFT* detector2 = new CSIFT();

	FREE_IMAGE_FORMAT fifmt = FreeImage_GetFileType(vImagePath1.c_str(), 0);
	FIBITMAP *dib = FreeImage_Load(fifmt, vImagePath1.c_str(), 0);

	FIBITMAP* greyScaleBitmap = FreeImage_ConvertToGreyscale(dib);
	BYTE *pixels = (BYTE*)FreeImage_GetBits(greyScaleBitmap);
	int width = FreeImage_GetWidth(greyScaleBitmap);
	int height = FreeImage_GetHeight(greyScaleBitmap);

	CImage<float> image1(width, height);

	for (size_t y = 0; y < height; y++)
	{
		for (size_t x = 0; x < width; x++)
		{
			image1.at(x, height - y - 1) = pixels[y*width + x] / 255.f;
		}
	}

	fifmt = FreeImage_GetFileType(vImagePath2.c_str(), 0);
	dib = FreeImage_Load(fifmt, vImagePath2.c_str(), 0);

	greyScaleBitmap = FreeImage_ConvertToGreyscale(dib);
	pixels = (BYTE*)FreeImage_GetBits(greyScaleBitmap);
	width = FreeImage_GetWidth(greyScaleBitmap);
	height = FreeImage_GetHeight(greyScaleBitmap);

	CImage<float> image2(width, height);

	for (size_t y = 0; y < height; y++)
	{
		for (size_t x = 0; x < width; x++)
		{
			image2.at(x, height - y - 1) = pixels[y*width + x] / 255.f;
		}
	}

	CImage<float>* input1 = downsampleBy2(downsampleBy2(downsampleBy2(&image1)));
	CImage<float>* input2 = downsampleBy2(downsampleBy2(downsampleBy2(&image2)));
	//input1->save("../../../my_test/1.png");
	//input2->save("../../../my_test/2.png");
	width = input1->ncols;
	height = input2->nrows;

	detector1->run(input1);
	detector2->run(input2);


	std::vector<std::pair<size_t, size_t>> matchResult;
	matchResult = matchSIFT(detector1, detector2);

	if (display) {
		cv::Mat companionImage(cv::Size(2 * width + 5, height), CV_8UC1, cv::Scalar(0));
		//std::cout << companionImage << std::endl;
		for (int y = 0; y < height; ++y) {
			for (int x = 0; x < width; x++)
			{
				companionImage.at<uchar>(y, x) = static_cast<int>(input1->at(x, y)*255.0f);
				companionImage.at<uchar>(y, x + width + 5) = static_cast<int>(input2->at(x, y)*255.0f);
			}
		}

		for (size_t i = 0; i < matchResult.size(); i++)
		{
			int x1, x2, y1, y2;
			x1 = static_cast<int>(detector1->keys[matchResult[i].first].x * detector1->keys[matchResult[i].first].delta);
			x2 = static_cast<int>(detector2->keys[matchResult[i].second].x * detector2->keys[matchResult[i].second].delta);
			y1 = static_cast<int>(detector1->keys[matchResult[i].first].y * detector1->keys[matchResult[i].first].delta);
			y2 = static_cast<int>(detector2->keys[matchResult[i].second].y * detector2->keys[matchResult[i].second].delta);

			x2 += width + 5;

			cv::line(companionImage, cv::Point(x1, y1), cv::Point(x2, y2)
				, cv::Scalar(0, 0, 255), 1, CV_AA);
		}

		cv::namedWindow("MyWindow", CV_WINDOW_NORMAL);
		cv::resizeWindow("MyWindow", cv::Size(1200, 600));
		cv::imshow("MyWindow", companionImage);
		cv::waitKey(0);
	}

	PairWiseCameras out;
	out.image1 = detector1->originalImage;
	out.image2 = detector2->originalImage;
	for (auto& item : matchResult) {
		float delta1 = detector1->keys[item.first].delta;
		float delta2 = detector2->keys[item.second].delta;
		float imageWidth1 = static_cast<float>(detector1->originalImage->ncols / delta1);
		float imageWidth2 = static_cast<float>(detector2->originalImage->ncols / delta2);
		float imageHeight1 = static_cast<float>(detector1->originalImage->nrows / delta1);
		float imageHeight2 = static_cast<float>(detector2->originalImage->nrows / delta2);

		out.matchResult.push_back(std::make_pair<glm::vec2, glm::vec2>(
			glm::vec2(detector1->keys[item.first].x / imageWidth1
				, detector1->keys[item.first].y / imageHeight1)
			, glm::vec2(detector2->keys[item.second].x / imageWidth2
				, detector2->keys[item.second].y / imageHeight2)));
	}
		

	return out;
}

Eigen::VectorXf estimate8Points(PairWiseCameras& pairCameras) {
	if (pairCameras.matchResult.size() < 8) {
		std::cout << "Not have enough point to estimate 8 point algorithm" << std::endl;
		return Eigen::VectorXf();
	}

	std::uniform_int_distribution<size_t> gen(0, pairCameras.matchResult.size()-1);
	std::mt19937 myRand(time(0));

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
	for (int i = 0; i < 8; ++i){
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

	Eigen::VectorXf solution = V.transpose().col(8);

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

double sampsonDistance(Eigen::Matrix<float, 3, 3> &vFoundamentalMatrix
	, std::pair<glm::vec2, glm::vec2> vCoordinates) {
	double p2_F_p1 = 0.0;
	p2_F_p1 += vCoordinates.second[0] * (vCoordinates.first[0] * vFoundamentalMatrix(0,0) 
		+ vCoordinates.first[1] * vFoundamentalMatrix(0,1) + vFoundamentalMatrix(0,2));
	p2_F_p1 += vCoordinates.second[1] * (vCoordinates.first[0] * vFoundamentalMatrix(1,0) 
		+ vCoordinates.first[1] * vFoundamentalMatrix(1,1) + vFoundamentalMatrix(1,2));
	p2_F_p1 += 1.0 * (vCoordinates.first[0] * vFoundamentalMatrix(2,0) 
		+ vCoordinates.first[1] * vFoundamentalMatrix(2,1) + vFoundamentalMatrix(2,2));
	p2_F_p1 *= p2_F_p1;

	double sum = 0.0;
	sum += pow(vCoordinates.first[0] * vFoundamentalMatrix(0,0) 
		+ vCoordinates.first[1] * vFoundamentalMatrix(0,1) + vFoundamentalMatrix(0,2), 2);
	sum += pow(vCoordinates.first[0] * vFoundamentalMatrix(1,0) 
		+ vCoordinates.first[1] * vFoundamentalMatrix(1,1) + vFoundamentalMatrix(1,2), 2);
	sum += pow(vCoordinates.second[0] * vFoundamentalMatrix(0,0) 
		+ vCoordinates.second[1] * vFoundamentalMatrix(1,0) + vFoundamentalMatrix(2,0), 2);
	sum += pow(vCoordinates.second[0] * vFoundamentalMatrix(0,1) 
		+ vCoordinates.second[1] * vFoundamentalMatrix(1,1) + vFoundamentalMatrix(2,1), 2);

	return p2_F_p1 / sum;
}

std::vector<size_t> findInlier(Eigen::Matrix<float, 3, 3> &vFoundamentalMatrix, PairWiseCameras& vPairCameras) {
	std::vector<size_t> result;
	result.resize(0);
	double const squared_thres = RANSAC_INLIER_THRESHOLD * RANSAC_INLIER_THRESHOLD;
	for (std::size_t i = 0; i < vPairCameras.matchResult.size(); ++i)
	{
		double error = sampsonDistance(vFoundamentalMatrix, vPairCameras.matchResult[i]);
		if (error < squared_thres)
			result.push_back(i);
	}
	return result;
}

void matchSfm(PairWiseCameras& pairCameras) {
	std::vector<size_t> inliers;
	inliers.reserve(pairCameras.matchResult.size());
	Eigen::Matrix<float, 3, 3> bestFoundamentalMatrix;
	for (size_t iter = 0; iter < 10000; iter++)
	{
		Eigen::VectorXf solution = estimate8Points(pairCameras);
		Eigen::Matrix<float, 3, 3> foundamentalMatrix = enforceConstrains(solution);
		std::vector<size_t> localInlier = findInlier(foundamentalMatrix, pairCameras);
		if (inliers.size() < localInlier.size()) {
			bestFoundamentalMatrix = foundamentalMatrix;
			inliers = localInlier;
		}
	}


	return;
}

int main(){
	PairWiseCameras t = getMatch("../../../my_test/00000.jpeg"
		, "../../../my_test/00001.jpeg");

	matchSfm(t);


    return 0;
}
