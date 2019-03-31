#include "CSIFT.h"
#include "FreeImage.h"
#include <iostream>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/xfeatures2d/nonfree.hpp>


void getMatch(const std::string vImagePath1, const std::string vImagePath2,bool display=true) {
	CSIFT detector1;
	CSIFT detector2;

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

	detector1.run(input1);
	detector2.run(input2);


	std::vector<std::pair<size_t, size_t>> matchResult;
	matchResult = matchSIFT(&detector1, &detector2);

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
			size_t x1, x2, y1, y2;
			x1 = detector1.keys[matchResult[i].first].x * detector1.keys[matchResult[i].first].delta;
			x2 = detector2.keys[matchResult[i].second].x * detector2.keys[matchResult[i].second].delta;
			y1 = detector1.keys[matchResult[i].first].y * detector1.keys[matchResult[i].first].delta;
			y2 = detector2.keys[matchResult[i].second].y * detector2.keys[matchResult[i].second].delta;

			x2 += width + 5;

			cv::line(companionImage, cv::Point(x1, y1), cv::Point(x2, y2)
				, cv::Scalar(0, 0, 255), 1, CV_AA);
		}

		cv::namedWindow("MyWindow", CV_WINDOW_NORMAL);
		cv::resizeWindow("MyWindow", cv::Size(1200, 600));
		cv::imshow("MyWindow", companionImage);
		cv::waitKey(0);
	}

	return;
}

int main(){
	getMatch("../../../my_test/00000.jpeg", "../../../my_test/00001.jpeg");

	


    return 0;
}
