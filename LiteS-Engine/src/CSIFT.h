#pragma once
#ifndef CSIFT_H
#define CSIFT_H

#include"CImage.h"
#include<vector>

struct SIFTParams {
	int nOctave=8;
	int imagesPerOctave=3;

	float sigma_min = 0.8;
	float delta_min = 0.5;

};

class CSIFT {
public:

	void scalespaceCompute(const CImage* vImage){
		float delta = this->param.delta_min;
		float sigma = this->param.sigma_min;

		for (size_t iOctave = 0; iOctave < param.nOctave; iOctave++) {
			for (size_t iScale = 0; iScale < param.imagesPerOctave; iScale++)
			{
				//First image in the each octave
				if (iScale == 0)
				{
					//First image in the first octave, needs upsample
					if (0 == iOctave)
					{
						CImage* imageAfterUpSampling = upsample(vImage,delta);
						CImage* imageAfterBlur = gassianBlur(imageAfterUpSampling, sigma);
					}
					//First image in the other octave, needs downsample
					else
					{
						CImage* imageAfterUpSampling = downsampleBy2(vImage);
						CImage* imageAfterBlur = gassianBlur(imageAfterUpSampling, sigma);
					}
				}
				
				//Blur

			}
		}
	}

	void scalespaceComputeDog() {

	}

	void run(const CImage* vImage) {
		
		scalespaceCompute(vImage);
		scalespaceComputeDog();
	}

	SIFTParams param;

	std::vector<CImage*> images;
};

#endif // 
