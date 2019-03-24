#pragma once
#ifndef CIMAGE_H
#define CIMAGE_H

#include<math.h>

void upsample(const CImage* vImage,float vDelta) {
	for (size_t y = 0; y < vImage->nrows; y++)
	{
		for (size_t x = 0; x < vImage->ncols; x++)
		{
			float xleftf = x * vDelta;
			float yleftf = y * vDelta;

			int xleft = static_cast<int>(xleftf);
			int yleft = static_cast<int>(yleftf);
			int xright = xleft + 1;
			int yright = yleft + 1;

			if (xleft > vImage->ncols)xleft = 2 * vImage->ncols - 1 - xleft;
			if (xright > vImage->ncols)xright = 2 * vImage->ncols - 1 - xright;
			if (yleft > vImage->nrows)yleft = 2 * vImage->nrows - 1 - yleft;
			if (yright > vImage->nrows)yright = 2 * vImage->nrows - 1 - yright;

			const float fractional_x = xleftf - floor(xleftf);
			const float fractional_y = yleftf - floor(yleftf);
			out[i*wo + j] = fractional_x * (fractional_y  * in[ip*wi + jp]
				+ (1 - fractional_y) * in[ip*wi + jm])
				+ (1 - fractional_x) * (fractional_y  * in[im*wi + jp]
					+ (1 - fractional_y) * in[im*wi + jm]);
		}
	}
}

class CImage{
public:
	int ncols;
	int nrows;
	float* data;

	float at(int vX, int vY) {
		if (data == nullptr)
			throw "Not init";
		return data[vY*ncols + vX];
	}



};

#endif // !CIMAGE_H
