#pragma once
#ifndef CIMAGE_H
#define CIMAGE_H

#include<math.h>

CImage* upsample(const CImage* vImage,float vDelta) {
	CImage* oImage;
	oImage->ncols = vImage->ncols / vDelta;
	oImage->nrows = vImage->nrows / vDelta;
	oImage->data = new float[oImage->ncols*oImage->nrows];
	for (size_t y = 0; y < oImage->nrows; y++)
	{
		for (size_t x = 0; x < oImage->ncols; x++)
		{
			float xleftf = x * vDelta;
			float yupf = y * vDelta;

			int xleft = static_cast<int>(xleftf);
			int yup = static_cast<int>(yupf);
			int xright = xleft + 1;
			int ydown = yup + 1;

			if (xleft > vImage->ncols)xleft = 2 * vImage->ncols - 1 - xleft;
			if (xright > vImage->ncols)xright = 2 * vImage->ncols - 1 - xright;
			if (yup > vImage->nrows)yup = 2 * vImage->nrows - 1 - yup;
			if (ydown > vImage->nrows)ydown = 2 * vImage->nrows - 1 - ydown;

			const float fractional_x = xleftf - floor(xleftf);
			const float fractional_y = yupf - floor(yupf);
			oImage->at(x,y)= fractional_x * (fractional_y  * vImage->at(xleft,yup)
				+ (1 - fractional_y) * vImage->at(xleft, ydown))
				+ (1 - fractional_x) * (fractional_y  * vImage->at(xleft, yup)
					+ (1 - fractional_y) * vImage->at(xleft, ydown));
		}
	}
	return oImage;
}

CImage* downsampleBy2(const CImage* vImage) {
	CImage* oImage = new CImage(vImage->ncols / 2,vImage->nrows/2);

	for (size_t iy = 0; iy < oImage->nrows; iy++) {
		for (size_t ix = 0; ix < oImage->ncols; ix++) {
			oImage->at(ix, iy) = vImage->at(ix * 2, iy * 2);
		}
	}

	return oImage;
}

CImage* gassianBlur(const CImage* vImage, float vSigma) {
	CImage* oImage;
	oImage->ncols = vImage->ncols;
	oImage->nrows = vImage->nrows;
	oImage->data = new float[oImage->ncols*oImage->nrows];
	
	for (size_t y = 0; y < oImage->nrows; y++)
	{
		for (size_t x = 0; x < oImage->ncols; x++)
		{
			
		}
		return oImage;
	}
}


class CImage{
public:
	int ncols;
	int nrows;
	float* data;

	CImage(const int vX, const int vY):ncols(vX),nrows(vY) {
		data = new float[nrows*ncols];
	}

	float& at(const int vX, const int vY) {
		if (data == nullptr)
			throw "Not init";
		return data[vY*ncols + vX];
	}

	float const& at(const int vX, const int vY) const {
		if (data == nullptr)
			throw "Not init";
		return data[vY*ncols + vX];
	}



};

#endif // !CIMAGE_H
