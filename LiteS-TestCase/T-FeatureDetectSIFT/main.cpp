#include "CSIFT.h"
#include "FreeImage.h"
#include <iostream>

int main(){
	CSIFT detector;

	FREE_IMAGE_FORMAT fifmt = FreeImage_GetFileType("../../../ny1-test/lena.jpg", 0);
	FIBITMAP *dib = FreeImage_Load(fifmt, "../../../ny1-test/lena.jpg", 0);
	
	FIBITMAP* greyScaleBitmap =FreeImage_ConvertToGreyscale(dib);
	BYTE *pixels = (BYTE*)FreeImage_GetBits(greyScaleBitmap);
	int width = FreeImage_GetWidth(greyScaleBitmap);
	int height = FreeImage_GetHeight(greyScaleBitmap);
	
	CImage<float> image(width, height);

	for (size_t y = 0; y < height; y++) 
	{
		for (size_t x = 0; x < width; x++)
		{
			image.at(x, height-y-1) = pixels[y*width + x]/255.f;
		}
	}

	detector.run(&image);

    return 0;
}
