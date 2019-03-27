#pragma once
#ifndef CIMAGE_H
#define CIMAGE_H

#include <cassert>
#include<math.h>
#include <iostream>

template<typename T>
class CImage{
public:
	int ncols;
	int nrows;
	T* data;

	CImage() :ncols(0), nrows(0) {}

	CImage(const size_t vX, const size_t vY):ncols(vX),nrows(vY) {
		data = new T[nrows*ncols];
	}

	T& at(const size_t vX, const size_t vY) {
		if (data == nullptr)
			throw "Not init";
		assert(vX >= 0 && vX < ncols&&vY >= 0 && vY < nrows);
		return data[vY*ncols + vX];
	}

	T const& at(const size_t vX, const size_t vY) const {
		if (data == nullptr)
			throw "Not init";
		assert(vX >= 0 && vX < ncols&&vY >= 0 && vY < nrows);
		return data[vY*ncols + vX];
	}

	T& at(const int vX, const int vY) {
		if (data == nullptr)
			throw "Not init";
		assert(vX >= 0 && vX < ncols&&vY >= 0 && vY < nrows);
		return data[vY*ncols + vX];
	}

	T const& at(const int vX, const int vY) const {
		if (data == nullptr)
			throw "Not init";
		assert(vX >= 0 && vX < ncols&&vY >= 0 && vY < nrows);
		return data[vY*ncols + vX];
	}

	static inline size_t symmetrizedCoordinates(int i, int l)
	{
		int ll = 2 * l;
		i = (i + ll) % (ll);
		if (i > l - 1) { i = ll - 1 - i; }
		return i;
	}

	CImage<float>* convoluted(const float* vxKernal, const size_t vxKernalSize
		, const float* vyKernal, const size_t vyKernalSize) const{
		CImage<float>* oImage=new CImage<float>(this->ncols, this->nrows);
		CImage<float>* tmpImage = new CImage<float>(this->ncols, this->nrows);

		// Along horizon
		for (int y = 0; y < tmpImage->nrows; y++)
		{
			for (int x = 0; x < tmpImage->ncols; x++)
			{
				float sum = this->at(x,y) * vxKernal[0];
				for (int k = 1; k < vxKernalSize; k++) {
					int t_left = symmetrizedCoordinates(x - k, tmpImage->ncols);
					int t_right = symmetrizedCoordinates(x + k, tmpImage->ncols);
					sum += vxKernal[k] * (this->at(t_left,y) + this->at(t_right, y));
				}
				tmpImage->at(x,y) = sum;
			}
		}
		// Along vertical
		for (int y = 0; y < oImage->nrows; y++)
		{
			for (int x = 0; x < oImage->ncols; x++)
			{
				float sum = this->at(x, y) * vyKernal[0];
				for (int k = 1; k < vyKernalSize; k++) {
					int t_up = symmetrizedCoordinates(y - k, oImage->nrows);
					int t_down = symmetrizedCoordinates(y + k, oImage->nrows);
					sum += vyKernal[k] * (tmpImage->at(x, t_up) + tmpImage->at(x, t_down));
				}
				oImage->at(x, y) = sum;
			}
		}

		return oImage;
	}

};

CImage<float>* upsample(const CImage<float>* vImage, float vDelta) {
	CImage<float>* oImage=new CImage<float>(vImage->ncols / vDelta, vImage->nrows / vDelta);
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

			if (xleft >= vImage->ncols)xleft = 2 * vImage->ncols - 1 - xleft;
			if (xright >= vImage->ncols)xright = 2 * vImage->ncols - 1 - xright;
			if (yup >= vImage->nrows)yup = 2 * vImage->nrows - 1 - yup;
			if (ydown >= vImage->nrows)ydown = 2 * vImage->nrows - 1 - ydown;

			const float fractional_x = xleftf - floor(xleftf);
			const float fractional_y = yupf - floor(yupf);
			oImage->at(x, y) = fractional_x * (fractional_y  * vImage->at(xleft, yup)
				+ (1. - fractional_y) * vImage->at(xleft, ydown))
				+ (1. - fractional_x) * (fractional_y  * vImage->at(xleft, yup)
					+ (1. - fractional_y) * vImage->at(xleft, ydown));
		}
	}
	return oImage;
}

CImage<float>* downsampleBy2(const CImage<float>* vImage) {
	CImage<float>* oImage = new CImage<float>(vImage->ncols / 2, vImage->nrows / 2);

	for (size_t iy = 0; iy < oImage->nrows; iy++) {
		for (size_t ix = 0; ix < oImage->ncols; ix++) {
			oImage->at(ix, iy) = vImage->at(ix * 2, iy * 2);
		}
	}

	return oImage;
}

float* getGussianKernal(float vSigma,size_t vKernalSize) {
	assert(vSigma > 0);
	assert(vKernalSize > 0);
	float* outKernal = new float[vKernalSize];
	outKernal[0] = 1.;
	float sum = outKernal[0];
	for (size_t i = 1; i < vKernalSize; i++)
	{
		outKernal[i] = exp(-0.5f * i * i / vSigma / vSigma);
		sum += 2 * outKernal[i];
	}
	for (size_t i = 0; i < vKernalSize; i++)
	{
		outKernal[i] /= sum;
	}
	return outKernal;
}


CImage<float>* gassianBlur(const CImage<float>* vImage, float vSigma) {
	int kernalRadius = (int)ceil(4 * vSigma) + 1;
	float* kernal = getGussianKernal(vSigma, kernalRadius);

	return vImage->convoluted(kernal, kernalRadius, kernal, kernalRadius);
}

template<int N>
void imagePatch(CImage<float>* img, int x, int y, float(*ptr)[N][N]) {
	int center = N / 2;
	assert(x >= center && x < img->ncols - center  && y >= center && y < img->nrows - center );
	assert(N % 2 != 0);
	assert(N < img->ncols&&N < img->nrows);
	for (int iy = 0; iy < N; ++iy) {
		for (int ix = 0; ix < N; ++ix)
		{
			(*ptr)[iy][ix] = img->at(x - center + ix, y - center + iy);
		}
	}
}


#endif // !CIMAGE_H
