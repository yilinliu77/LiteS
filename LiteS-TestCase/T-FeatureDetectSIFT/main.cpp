#include "CSIFT.h"
#include "FreeImage.h"
#include <iostream>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/xfeatures2d/nonfree.hpp>
int main(){
	CSIFT detector1;
	CSIFT detector2;

	FREE_IMAGE_FORMAT fifmt = FreeImage_GetFileType("../../../my_test/00000.jpeg", 0);
	FIBITMAP *dib = FreeImage_Load(fifmt, "../../../my_test/00000.jpeg", 0);

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

	fifmt = FreeImage_GetFileType("../../../my_test/00001.jpeg", 0);
	dib = FreeImage_Load(fifmt, "../../../my_test/00001.jpeg", 0);

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
	input1->save("../../../my_test/1.png");
	input2->save("../../../my_test/2.png");
	width = input1->ncols;
	height = input2->nrows;

	cv::Mat companionImage(cv::Size(2 * width + 5, height), CV_8UC1, cv::Scalar(0));
	//std::cout << companionImage << std::endl;
	for (int y = 0; y < height; ++y) {
		for (int x = 0; x < width; x++)
		{
			companionImage.at<uchar>(y, x) = static_cast<int>(input1->at(x, y)*255.0f);
			companionImage.at<uchar>(y,x + width + 5) = static_cast<int>(input2->at(x, y)*255.0f);
		}
	}


	detector1.run(input1);
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
		if (matchItems[i][0].distance / matchItems[i][1].distance <= 0.6)
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
