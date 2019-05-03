#include "CSIFT.h"
#include "FreeImage.h"
#include <iostream>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/xfeatures2d/nonfree.hpp>
#include "main.h"

void visualizeDetector(CSIFT& vDetector) {
	cv::Mat cvImage1(cv::Size(vDetector.originalImage->ncols
		, vDetector.originalImage->nrows), CV_8UC1, cv::Scalar(0));
	for (int y = 0; y < vDetector.originalImage->nrows; ++y) {
		for (int x = 0; x < vDetector.originalImage->ncols; x++) {
			cvImage1.at<uchar>(y, x) 
				= static_cast<uchar>(vDetector.originalImage->at(x, y)*255.0f);
		}
	}
	//For OpenCV
	cv::Ptr<cv::Feature2D> sift = cv::xfeatures2d::SIFT::create(1600);
	std::vector<cv::KeyPoint> keypoints1;
	keypoints1.clear();
	cv::Mat a1;
	sift->detectAndCompute(cvImage1, cv::Mat(), keypoints1, a1);
	//cv::drawKeypoints(cvImage1, keypoints1, cvImage1);

	cv::Mat companionImage(cv::Size(2 * vDetector.originalImage->ncols + 5
		, vDetector.originalImage->nrows), CV_8UC1, cv::Scalar(0));
	for (int y = 0; y < vDetector.originalImage->nrows; ++y) {
		for (int x = 0; x < vDetector.originalImage->ncols; x++) {
			companionImage.at<uchar>(y, x)
				= static_cast<int>(vDetector.originalImage->at(x, y)*255.0f);

			companionImage.at<uchar>(y, x + 5 + vDetector.originalImage->ncols)
				= static_cast<int>(cvImage1.at<uchar>(y, x));
		}
	}

	for (const auto& key: vDetector.keys) {
		cv::circle(companionImage, cv::Point(key.x*key.delta, key.y *key.delta)
			, 2, cv::Scalar(0));
	}

	for (const auto& keyItem : keypoints1) {
		cv::circle(companionImage
			, cv::Point(keyItem.pt.x + 5 + vDetector.originalImage->ncols
				, keyItem.pt.y)
			, 2, cv::Scalar(0));
	}

	cv::namedWindow("MyWindow", CV_WINDOW_NORMAL);
	cv::resizeWindow("MyWindow", cv::Size(1200, 600));
	
	cv::imshow("MyWindow", companionImage);
	cv::waitKey(0);
}

int main(){
	CSIFT detector1;
	CSIFT detector2;

	FREE_IMAGE_FORMAT fifmt = FreeImage_GetFileType("../../../my_test/test_low/1.png", 0);
	FIBITMAP *dib = FreeImage_Load(fifmt, "../../../my_test/test_low/1.png", 0);

	// Convert non-32 bit images
	if (FreeImage_GetBPP(dib) != 32) {
		FIBITMAP* hOldImage = dib;
		dib = FreeImage_ConvertTo32Bits(hOldImage);
		FreeImage_Unload(hOldImage);
	}

	FIBITMAP* greyScaleBitmap = FreeImage_ConvertToGreyscale(dib);
	BYTE *pixels = (BYTE*)FreeImage_GetBits(greyScaleBitmap);
	int width = FreeImage_GetWidth(greyScaleBitmap);
	int height = FreeImage_GetHeight(greyScaleBitmap);

	CImage<float> image1(width, height);

	for (int y = 0; y < height; ++y) {
		BYTE* pixelScanline = FreeImage_GetScanLine(greyScaleBitmap, y);
		for (int x = 0; x < width; x++) {
			image1.at(x, height - y - 1) = pixelScanline[x] / 255.f;
		}
	}

	fifmt = FreeImage_GetFileType("../../../my_test/test_low/3.png", 0);
	dib = FreeImage_Load(fifmt, "../../../my_test/test_low/3.png", 0);

	// Convert non-32 bit images
	if (FreeImage_GetBPP(dib) != 32) {
		FIBITMAP* hOldImage = dib;
		dib = FreeImage_ConvertTo32Bits(hOldImage);
		FreeImage_Unload(hOldImage);
	}

	greyScaleBitmap = FreeImage_ConvertToGreyscale(dib);
	pixels = (BYTE*)FreeImage_GetBits(greyScaleBitmap);
	width = FreeImage_GetWidth(greyScaleBitmap);
	height = FreeImage_GetHeight(greyScaleBitmap);

	CImage<float> image2(width, height);

	for (int y = 0; y < height; ++y) {
		BYTE* pixelScanline = FreeImage_GetScanLine(greyScaleBitmap, y);
		for (int x = 0; x < width; x++) {
			image2.at(x, height - y - 1) = pixelScanline[x] / 255.f;
		}
	}

	CImage<float>* input1 = &image1;
	CImage<float>* input2 = &image2;
	//input1->save("../../../my_test/1.png");
	//input2->save("../../../my_test/2.png");
	width = input1->ncols;
	height = input2->nrows;

	cv::Mat companionImage(cv::Size(2 * width + 5, height * 2 + 5)
		, CV_8UC1, cv::Scalar(0));
	//std::cout << companionImage << std::endl;
	for (int y = 0; y < height; ++y) {
		for (int x = 0; x < width; x++)
		{
			companionImage.at<uchar>(y, x) = static_cast<int>(input1->at(x, y)*255.0f);
			companionImage.at<uchar>(y + height + 5,x + width + 5) = static_cast<int>(input2->at(x, y)*255.0f);
		}
	}


	detector1.run(input1);

	//visualizeDetector(detector1);

	detector2.run(input2);

	std::vector<std::pair<size_t, size_t>> matchResult;
	matchResult = matchSIFT(&detector1, &detector2);

	for (size_t i = 0; i < matchResult.size(); i++)
	{
		size_t x1, x2, y1, y2;
		x1 = detector1.keys[matchResult[i].first].x * detector1.keys[matchResult[i].first].delta;
		x2 = detector2.keys[matchResult[i].second].x * detector2.keys[matchResult[i].second].delta;
		y1 = detector1.keys[matchResult[i].first].y * detector1.keys[matchResult[i].first].delta;
		y2 = detector2.keys[matchResult[i].second].y * detector2.keys[matchResult[i].second].delta;

		x2 += width + 5;
		y2 += height + 5;

		cv::line(companionImage, cv::Point(x1, y1), cv::Point(x2, y2)
			, cv::Scalar(0, 0, 255),1, CV_AA);
	}

	cv::namedWindow("MyWindow", CV_WINDOW_NORMAL);
	cv::resizeWindow("MyWindow", cv::Size(1200, 600));
	cv::imshow("MyWindow", companionImage);
	cv::waitKey(0);

	//SIFT in opencv
	cv::Mat cvImage1(cv::Size(input1->ncols, input1->nrows), CV_8UC1, cv::Scalar(0));
	cv::Mat cvImage2(cv::Size(input2->ncols, input2->nrows), CV_8UC1, cv::Scalar(0));
	for (int y = 0; y < input1->nrows; ++y) {
		for (int x = 0; x < input1->ncols; x++)
		{
			cvImage1.at<uchar>(y, x) = static_cast<int>(input1->at(x, y)*255.0f);
			cvImage2.at<uchar>(y, x) = static_cast<int>(input2->at(x, y)*255.0f);

		}
	}

	cv::Ptr<cv::Feature2D> sift = cv::xfeatures2d::SIFT::create(1600);
	std::vector<cv::KeyPoint> keypoints1;
	std::vector<cv::KeyPoint> keypoints2;
	keypoints1.clear();
	keypoints2.clear();
	cv::Mat a1;
	cv::Mat a2;
	sift->detectAndCompute(cvImage1, cv::Mat(), keypoints1, a1);
	sift->detectAndCompute(cvImage2, cv::Mat(), keypoints2, a2);

	cv::FlannBasedMatcher matcher;
	std::vector <std::vector<cv::DMatch>> matchItems;
	matcher.knnMatch(a1,a2,matchItems,2);
	double max_dist = 0; double min_dist = 100;

	std::vector<cv::DMatch> good_matches;
	int q = 0;
	for(int i = 0; i < a1.rows; i ++)
	{ 
		if (matchItems[i][0].distance / matchItems[i][1].distance <= MATCHTHRESHOLD)
		{
			good_matches.push_back(matchItems[i][0]);
		}
		else
			int a = 1;
	}
	cv::Mat outImage;
	cv::drawMatches(cvImage1, keypoints1, cvImage2, keypoints2, good_matches, outImage);
	//cv::drawKeypoints(cvImage1, keypoints1, cvImage1);
	cv::imshow("output", outImage);
	cv::waitKey(0);


    return 0;
}
