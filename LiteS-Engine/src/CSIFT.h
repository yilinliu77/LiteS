#pragma once
#ifndef CSIFT_H
#define CSIFT_H

#include"CImage.h"
#include<vector>
#include <algorithm>

struct SIFTParams {
	int nOctave=8;
	int imagesPerOctave=3;

	float sigma_min = 1.6f;
	float delta_min = 0.5f;

	float sigma_in = 0.5f;

};

struct Keypoint {
	int x;
	int y;
	float value;
	int scale;
	int octave;
	float sigma;

};

class CSIFT {
public:
	float *delta;
	float **sigma;
	std::vector<std::vector<CImage<float>*>> images;
	std::vector<std::vector<CImage<float>*>> dogImages;
	std::vector<Keypoint> keys;
	SIFTParams param;

	void init(int vW,int vH) {
		// The minimal size (width or height) of images in the last octave.
		int hmin = 32;
		// The size (min of width and height) of images in the first octave.
		int h0 = float(std::min(vW, vH)) / this->param.delta_min;
		// The number of octaves. 0.693147180559945309417==M_LN2
		int n_oct = std::min(this->param.nOctave, (int)(std::log(h0 / hmin) / 0.693147180559945309417) + 1);
		this->param.nOctave = n_oct;

		delta = new float[this->param.nOctave];
		sigma = new float*[this->param.nOctave];

		delta[0] = this->param.delta_min;
		for (size_t iOctave = 0; iOctave < param.nOctave; iOctave++) {
			if (0 != iOctave) {
				delta[iOctave] = delta[iOctave - 1] * 2;
			}
			sigma[iOctave] = new float[param.imagesPerOctave+3];
			for (size_t iScale = 0; iScale < param.imagesPerOctave+3; iScale++) {
				sigma[iOctave][iScale] = delta[iOctave] / this->param.delta_min * this->param.sigma_min
					* static_cast<float>(std::pow(2, float(iScale) / float(this->param.imagesPerOctave)));
			}
		}
	}

	void scalespaceCompute(const CImage<float>* vImage){

		for (size_t iOctave = 0; iOctave < param.nOctave; iOctave++) {
			for (size_t iScale = 0; iScale < param.imagesPerOctave + 3; iScale++)
			{
				//First image in the each octave
				if (iScale == 0)
				{
					images.push_back(std::vector<CImage<float>*>());
					//First image in the first octave, needs upsample
					if (0 == iOctave)
					{
						float currentSigma = std::sqrt(this->param.sigma_min*this->param.sigma_min 
							- 4 * this->param.sigma_in * this->param.sigma_in);
						CImage<float>* imageAfterUpSampling = upsample(vImage,this->delta[0]);
						CImage<float>* imageAfterBlur = gassianBlur(imageAfterUpSampling, currentSigma);

						images[iOctave].push_back(imageAfterBlur);
					}
					//First image in the other octave, needs downsample
					else
					{
						CImage<float>* imageAfterDownSampling = downsampleBy2(images[iOctave-1][this->param.imagesPerOctave]);
						images[iOctave].push_back(imageAfterDownSampling);
					}
				}
				else {
					float currentSigma = std::sqrt(sigma[iOctave][iScale] * sigma[iOctave][iScale]
						- sigma[iOctave][iScale-1] * sigma[iOctave][iScale-1]);
					CImage<float>* imageAfterBlur = gassianBlur(images[iOctave][iScale-1]
						, currentSigma);
					images[iOctave].push_back(imageAfterBlur);
				}
			}
		}
	}

	void scalespaceComputeDog() {
		for (size_t iOctave = 0; iOctave < param.nOctave; iOctave++) {
			dogImages.push_back(std::vector<CImage<float>*>());
			for (size_t iScale = 0; iScale < param.imagesPerOctave+2; iScale++) {
				CImage<float>* dogImage = new CImage<float>(images[iOctave][0]->ncols, images[iOctave][0]->nrows);
				for (int y = 0; y < dogImage->nrows; y++)
					for (int x = 0; x < dogImage->ncols; x++)
						dogImage->at(x, y) = images[iOctave][iScale + 1]->at(x, y) 
						- images[iOctave][iScale]->at(x, y);
				dogImages[iOctave].push_back(dogImage);
			}
		}
	}

	void extractExtrema() {
		for (int iOctave = 0; iOctave < this->param.nOctave; iOctave++) {
			// Loop through the samples of the image stack (one octave)
			for (int iScale = 1; iScale < images[iOctave].size() - 1; iScale++) {
				for (int y = 1; y < images[iOctave][0]->nrows - 1; y++) {
					for (int x = 1; x < images[iOctave][0]->ncols - 1; x++) {

						float patch1[9];
						float patch2[9];
						float patch3[9];
						imagePatch<3>(images[iOctave][iScale-1], x, y, (float(*)[3][3])&patch1);
						imagePatch<3>(images[iOctave][iScale], x, y, (float(*)[3][3])&patch2);
						imagePatch<3>(images[iOctave][iScale+1], x, y, (float(*)[3][3])&patch3);

						float centerValue = patch2[4];
						bool isLocalMin = true;
						// An optimizing compiler will unroll this loop.
						for (int n = 0; n < 9; n++) {
							if (patch1[n] - 0.00001 <= centerValue) {
								isLocalMin = false;
								break; // Can stop early if a smaller neighbor was found.
							}
							if (patch3[n] - 0.00001 <= centerValue) {
								isLocalMin = false;
								break; 
							}
						}
						bool isLocalMax = true;
						// Can skip max check if center point was determined to be a local min.
						if (isLocalMin) {
							isLocalMax = false;
						}
						else {
							for (int n = 0; n < 9; n++) {
								if (patch1[n] - 0.00001 >= centerValue) {
									isLocalMax = false;
									break; // Can stop early if a smaller neighbor was found.
								}
								if (patch3[n] - 0.00001 >= centerValue) {
									isLocalMax = false;
									break;
								}
							}
						}
						if (isLocalMax || isLocalMin) {
							Keypoint key;
							key.scale = iScale;
							key.octave = iOctave;
							key.x = x;
							key.y = y;
							key.sigma = sigma[iOctave][iScale];
							key.value = centerValue;
							keys.push_back(key);
						}
					}
				}
			}
		}
	}

	void run(const CImage<float>* vImage) {
		init(vImage->ncols,vImage->nrows);
		scalespaceCompute(vImage);
		scalespaceComputeDog();
		extractExtrema();
	}



};

#endif // 
