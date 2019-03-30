#include "CSIFT.h"
#include "FreeImage.h"
#include <iostream>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"

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

		cv::line(companionImage,cv::Point)
	}

	cv::namedWindow("MyWindow", CV_WINDOW_AUTOSIZE);
	cv::imshow("MyWindow", companionImage);
	cv::waitKey(0);
	cv::destroyWindow("MyWindow");

    return 0;
}
