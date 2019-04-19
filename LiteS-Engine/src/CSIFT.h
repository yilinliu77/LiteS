#pragma once
#ifndef CSIFT_H
#define CSIFT_H

#include"CImage.h"
#include<vector>
#include <algorithm>
#include <FreeImage.h>
#include <iostream>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/xfeatures2d/nonfree.hpp>

#define EPSILON 0
#define MATCHTHRESHOLD 0.6

struct SIFTParams {
	int nOctave=8;
	int imagesPerOctave=3;

	float sigma_min = 0.8f;
	float delta_min = 0.5f;

	float sigma_in = 0.5f;

	float C_Dog = 0.013333333f;
	float keysThreshold = 0.013333333f;

	float edgeThreshold =10;

	int nbins = 36;
	float lambda_ori = 1.5;
	float lambda_descr = 6.0f;

	int n_hist = 4;
	int n_ori = 8;
};

struct Keypoint {
	int x;
	int y;
	float value;
	int scale;
	int octave;
	float sigma;
	float delta;
	std::vector<float> hist;
	std::vector<float> desc;
	float theta=-1;
};

class CSIFT {
public:
	float *delta;
	float **sigma;
	std::vector<std::vector<CImage<float>*>> images;
	std::vector<std::vector<CImage<float>*>> dogImages;
	std::vector<std::vector<CImage<float>*>> gradientX;
	std::vector<std::vector<CImage<float>*>> gradientY;
	std::vector<Keypoint> keys;
	SIFTParams param;
	CImage<float>* originalImage;

	void init(const int vW, const int vH) {
		// The minimal size (width or height) of images in the last octave.
		int hmin = 32;
		// The size (min of width and height) of images in the first octave.
		int h0 = static_cast<int>(float(std::min(vW, vH)) / this->param.delta_min);
		// The number of octaves. 0.693147180559945309417==M_LN2
		int n_oct = std::min(this->param.nOctave, (int)(std::log(h0 / hmin) / 0.693147180559945309417f) + 1);
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
		//0.693147180559945309417==ln2
		float k_nspo = exp(0.693147180559945309417f / ((float)(param.imagesPerOctave)));
		float k_3 = exp(0.693147180559945309417f / ((float)3));
		param.keysThreshold = (k_nspo - 1) / (k_3 - 1) * param.C_Dog; 
		param.edgeThreshold = (param.edgeThreshold)*(param.edgeThreshold + 1) / param.edgeThreshold;
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
							- this->param.sigma_in * this->param.sigma_in)/delta[iOctave];
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
						- sigma[iOctave][iScale-1] * sigma[iOctave][iScale-1])/delta[iOctave];
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
			for (int iScale = 1; iScale < dogImages[iOctave].size() - 1; iScale++) {
				for (int y = 1; y < dogImages[iOctave][0]->nrows - 1; y++) {
					for (int x = 1; x < dogImages[iOctave][0]->ncols - 1; x++) {

						float patch1[9];
						float patch2[9];
						float patch3[9];
						imagePatch<3>(dogImages[iOctave][iScale-1], x, y, (float(*)[3][3])&patch1);
						imagePatch<3>(dogImages[iOctave][iScale], x, y, (float(*)[3][3])&patch2);
						imagePatch<3>(dogImages[iOctave][iScale+1], x, y, (float(*)[3][3])&patch3);

						float centerValue = patch2[4];
						bool isLocalMin = true;
						// An optimizing compiler will unroll this loop.
						for (int n = 0; n < 9; n++) {
							if (patch1[n] - EPSILON <= centerValue) {
								isLocalMin = false;
								break; // Can stop early if a smaller neighbor was found.
							}
							if (patch2[n] - EPSILON <= centerValue && n!=4) {
								isLocalMin = false;
								break; // Can stop early if a smaller neighbor was found.
							}
							if (patch3[n] - EPSILON <= centerValue) {
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
								if (patch1[n] - EPSILON >= centerValue) {
									isLocalMax = false;
									break; // Can stop early if a smaller neighbor was found.
								}
								if (patch2[n] - EPSILON >= centerValue && n != 4) {
									isLocalMax = false;
									break; // Can stop early if a smaller neighbor was found.
								}
								if (patch3[n] - EPSILON >= centerValue) {
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
							key.delta = delta[iOctave];
							key.sigma = sigma[iOctave][iScale];
							key.value = centerValue;
							keys.push_back(key);
						}
					}
				}
			}
		}
	}

	void taylorExpand(float *vOffsetX, float *vOffsetY, float *vOffsetS,float *vValue
		,int x,int y,int s,int o) {
		float hXX, hXY, hXS, hYY, hYS, hSS;
		float det, aa, ab, ac, bb, bc, cc;
		float gX, gY, gS;
		float ofstX, ofstY, ofstS, ofstVal;

		/** Compute the 3d Hessian at pixel (i,j,s)  Finite difference scheme  *****/
		hXX = images[o][s]->at(x - 1, y) + images[o][s]->at(x + 1, y) - 2 * images[o][s]->at(x, y);
		hYY = images[o][s]->at(x, y + 1) + images[o][s]->at(x, y - 1) - 2 * images[o][s]->at(x, y);
		hSS = images[o][s + 1]->at(x, y) + images[o][s - 1]->at(x, y) - 2 * images[o][s]->at(x, y);
		hXY = 0.25f*((images[o][s]->at(x + 1, y + 1) - images[o][s]->at(x, y - 1))
			- (images[o][s]->at(x - 1, y) - images[o][s]->at(x - 1, y - 1)));
		hXS = 0.25f*((images[o][s + 1]->at(x + 1, y) - images[o][s + 1]->at(x - 1, y))
			- (images[o][s - 1]->at(x + 1, y) - images[o][s - 1]->at(x - 1, y)));
		hYS = 0.25f*((images[o][s + 1]->at(x + 1, y) - images[o][s + 1]->at(x, y - 1))
			- (images[o][s - 1]->at(x, y + 1) - images[o][s - 1]->at(x, y - 1)));

		/** Compute the 3d gradient at pixel (i,j,s) */
		gX = 0.5f*(images[o][s]->at(x+1, y) - images[o][s]->at(x - 1, y));
		gY = 0.5f*(images[o][s]->at(x , y + 1) - images[o][s]->at(x , y - 1));
		gS = 0.5f*(images[o][s + 1]->at(x, y ) - images[o][s - 1]->at(x, y ));

		/** Inverse the Hessian - Fitting a quadratic function */
		det = hXX * hYY*hSS - hXX * hYS*hYS - hXY * hXY*hSS + 2 * hXY*hXS*hYS - hXS * hXS*hYY;
		aa = (hYY*hSS - hYS * hYS) / det;
		ab = (hXS*hYS - hXY * hSS) / det;
		ac = (hXY*hYS - hXS * hYY) / det;
		bb = (hXX*hSS - hXS * hXS) / det;
		bc = (hXY*hXS - hXX * hYS) / det;
		cc = (hXX*hYY - hXY * hXY) / det;

		// offset
		ofstX = -aa * gX - ab * gY - ac * gS; // in position
		ofstY = -ab * gX - bb * gY - bc * gS;
		ofstS = -ac * gX - bc * gY - cc * gS;
		/** Compute the DoG value offset */
		ofstVal = +0.5f*(gX*ofstX + gY * ofstY + gS * ofstS); //... and value

		// output
		*vOffsetX = ofstX;
		*vOffsetY = ofstY;
		*vOffsetS = ofstS;
		*vValue = images[o][s]->at(x, y) + ofstVal;
	}

	void refineKeyPoint() {
		for (int ik = 0; ik < keys.size(); ik++) {
			size_t iter = 0;
			const size_t MAX_ITER = 5;
			const float MAXOFFSET = 0.6f;
			bool isConf = false;

			int s = keys[ik].scale;
			int x = keys[ik].x;
			int y = keys[ik].y;
			int o = keys[ik].octave;
			float value = images[o][s]->at(x, y);

			const int maxWidth = images[o][0]->ncols;
			const int maxHeight = images[o][0]->nrows;
			const int maxScale = static_cast<int>(images[o].size());

			while (iter<MAX_ITER)
			{
				float offsetX, offsetY, offsetS;

				taylorExpand(&offsetX, &offsetY, &offsetS, &value
					, x, y, s, o);

				// Test if the quadratic model is consistent 
				if ((abs(offsetX) < MAXOFFSET) && (abs(offsetY) < MAXOFFSET) && (abs(offsetS) < MAXOFFSET)) {
					isConf = true;
					break;
				}
				else { // move to another point
					if ((offsetX > +MAXOFFSET) && ((x + 1) < (maxWidth- 1))) { x += 1; }
					if ((offsetX < -MAXOFFSET) && ((x - 1) > 0)) { x -= 1; }
					if ((offsetY > +MAXOFFSET) && ((y + 1) < (maxHeight - 1))) { y += 1; }
					if ((offsetY < -MAXOFFSET) && ((y - 1) > 0)) { y -= 1; }

					if ((offsetS > +MAXOFFSET) && ((s + 1) < (maxScale - 1))) { s += 1; }
					if ((offsetS < -MAXOFFSET) && ((s - 1) > 0)) { s -= 1; }
				}

				iter += 1;
			}

			if (isConf) {
				keys[ik].x = x;
				keys[ik].y = y;
				keys[ik].scale = s;
				keys[ik].value = value;
			}
			else {
				keys.erase(keys.begin() + ik);
			}
		}
	}

	void discardKeysWithLowContrast(float vThreshold) {
		for (int ik = 0; ik < keys.size(); ik++) {
			if (abs(keys[ik].value) < vThreshold)
				keys.erase(keys.begin() + ik);
		}
	}

	void discardKeysOnEdge() {
		for (int ik = 0; ik < keys.size(); ik++) {
			// Loading keypoint and associated octave
			int o = keys[ik].octave;
			int s = keys[ik].scale;
			int x = keys[ik].x;
			int y = keys[ik].y;
			// Compute the 2d Hessian at pixel (i,j)
			float hXX = images[o][s]->at(x-1,y) + images[o][s]->at(x + 1, y) - 2 * images[o][s]->at(x, y);
			float hYY = images[o][s]->at(x , y - 1) + images[o][s]->at(x , y + 1) - 2 * images[o][s]->at(x, y);
			float hXY = 1.f / 4.f * ((images[o][s]->at(x+1, y + 1) - images[o][s]->at(x+1, y-1)) - (images[o][s]->at(x-1, y + 1) - images[o][s]->at(x-1, y - 1)));
			// Harris and Stephen Edge response
			float edgeResp = (hXX + hYY)*(hXX + hYY) / (hXX*hYY - hXY * hXY);
			if (abs(edgeResp) > param.edgeThreshold)
				keys.erase(keys.begin() + ik);
		}
	}

	void preComputerGradient() {
		for (int iOctave = 0; iOctave < this->param.nOctave; iOctave++) {
			gradientX.push_back(std::vector<CImage<float>*>());
			gradientY.push_back(std::vector<CImage<float>*>());
			for (int iScale = 0; iScale < images[iOctave].size(); iScale++) {
				CImage<float>* dx = new CImage<float>(images[iOctave][0]->ncols, images[iOctave][0]->nrows);
				CImage<float>* dy = new CImage<float>(images[iOctave][0]->ncols, images[iOctave][0]->nrows);
				for (int y = 0; y < images[iOctave][0]->nrows; y++) {
					for (int x = 0; x < images[iOctave][0]->ncols; x++) {
						if (x == 0 || y == 0 || x == images[iOctave][0]->ncols - 1 || y == images[iOctave][0]->nrows - 1)
						{
							dx->at(x, y) = 0;
							dy->at(x, y) = 0;
						}
						else {
							dx->at(x, y) = (images[iOctave][iScale]->at(x + 1, y) - images[iOctave][iScale]->at(x - 1, y))/2.0f;
							dy->at(x, y) = (images[iOctave][iScale]->at(x , y + 1) - images[iOctave][iScale]->at(x , y - 1))/2.0f;
						}
					}
				}
				gradientX[iOctave].push_back(dx);
				gradientY[iOctave].push_back(dy);
			}
		}
	}

	void computeOrientation() {
		std::vector<Keypoint> t_keys;
		for (int ik = 0; ik < keys.size(); ik++) {
			// Initialize output vector
			for (int i = 0; i < param.nbins; i++) { keys[ik].hist.push_back(0.0f); }

			float sigmaKey = keys[ik].sigma / delta[keys[ik].octave];
			// Contributing pixels are inside a patch [sxMin;sxMax] X [syMin;syMax]
			// of width w 6*lambda_ori*sigmaKey (=9*sigmaKey)
			float R = 3 * param.lambda_ori*sigmaKey;
			int sxMin = std::max(0, (int)(keys[ik].x - R + 0.5));
			int syMin = std::max(0, (int)(keys[ik].y - R + 0.5));
			int sxMax = std::min((int)(keys[ik].x + R + 0.5), images[keys[ik].octave][0]->ncols - 1);
			int syMax = std::min((int)(keys[ik].y + R + 0.5), images[keys[ik].octave][0]->nrows - 1);

			/// For each pixel inside the patch.
			for (int sy = syMin; sy <= syMax; sy++) {
				for (int sx = sxMin; sx <= sxMax; sx++) {
					// Compute pixel coordinates (sX,sY) on keypoint's invariant
					//referential.
					float sX = (sx - keys[ik].x) / sigmaKey;
					float sY = (sy - keys[ik].y) / sigmaKey;

					// gradient orientation (theta)
					float dx = gradientX[keys[ik].octave][keys[ik].scale]->at(sx,sy);
					float dy = gradientY[keys[ik].octave][keys[ik].scale]->at(sx, sy);
					float ori = atan2(dy, dx);

					// gradient magnitude with Gaussian weighing
					float r2 = sX * sX + sY * sY;
					float M = hypot(dx, dy) * exp(-r2 / (2 * param.lambda_ori*param.lambda_ori));

					/// Determine the bin index in the circular histogram
					if (ori < 0)
						ori += 2.f * 3.1415926f;
					int gamma = (int)(ori / (2.f * 3.1415926f)*param.nbins + 0.5f) % param.nbins;

					/// Add the contribution to the orientation histogram
					keys[ik].hist[gamma] += M;
				}
			}

			// Extract principal orientation
			// Smooth histogram : 6 iterated box filters
			int i, i_prev, i_next;
			float *tmp = new float[param.nbins];
			/// Initialization
			for (i = 0; i < param.nbins; i++)
				tmp[i] = keys[ik].hist[i];
			// Convolution with box filters
			int niter = 6;
			for (; niter > 0; niter--) {
				for (i = 0; i < param.nbins; i++)
					tmp[i] = keys[ik].hist[i];
				for (i = 0; i < param.nbins; i++) {
					i_prev = (i - 1 + param.nbins) % param.nbins;
					i_next = (i + 1) % param.nbins;
					keys[ik].hist[i] = (tmp[i_prev] + tmp[i] + tmp[i_next]) / 3.f;
				}
			}
			// What is the value of the global maximum
			float max_value = *std::max_element(keys[ik].hist.begin(), keys[ik].hist.end());
			// Search for local extrema in the histogram
			for (int i = 0; i < param.nbins; i++) {
				int i_prev = (i - 1 + param.nbins) % param.nbins;
				int i_next = (i + 1) % param.nbins;
				if ((keys[ik].hist[i] > 0.8*max_value) 
					&& (keys[ik].hist[i] > keys[ik].hist[i_prev]) 
					&& (keys[ik].hist[i] > keys[ik].hist[i_next])) {
					// Quadratic interpolation of the position of each local maximum
					float offset = (keys[ik].hist[i_prev] - keys[ik].hist[i_next])
						/ (2 * (keys[ik].hist[i_prev] + keys[ik].hist[i_next] - 2 * keys[ik].hist[i]));
					// Add to vector of principal orientations (expressed in [0,2pi]
					float ori = ((float)i + offset + 0.5f) * 2.f * 3.1415926f / (float)param.nbins;
					if (ori > 3.1415926f)
						ori -= 2.f * 3.1415926f;
					Keypoint tk(keys[ik]);
					tk.theta = ori;
					t_keys.push_back(tk);
				}
			}
		}


		keys = t_keys;
	}

	void applyRotation(float x, float y, float *rx, float *ry, float alpha)
	{
		float c = cos(alpha);
		float s = sin(alpha);
		float tx = c * x - s * y;
		float ty = s * x + c * y;
		*rx = tx;
		*ry = ty;
	}

	float applyL2Norm(const std::vector<float> array, int length) {
		float l2norm = 0;
		for (int i = 0; i < length; i++) {
			l2norm += array[i] * array[i];
		}
		l2norm = sqrt(l2norm);
		return l2norm;
	}

	void computeDescriptor() {
		int n_descr = param.n_hist * param.n_hist * param.n_ori;

		for (int ik = 0; ik < keys.size(); ik++) {

			/** Loading keypoint gradient scalespaces */
			float x = static_cast<float>(keys[ik].x);
			float y = static_cast<float>(keys[ik].y);
			int o = keys[ik].octave;
			int s = keys[ik].scale;
			float sigma = keys[ik].sigma;
			float theta = keys[ik].theta;

			// load scalespace gradient
			int w = images[o][0]->ncols;
			int h = images[o][0]->nrows;
			float currentDelta = delta[o];

			sigma /= currentDelta;

			// Compute descriptor representation
			// Initialize descriptor tab
			for (int i = 0; i < n_descr; i++) { keys[ik].desc.push_back(0.f); }
			// Contributing pixels are inside a patch [siMin;siMax]X[sjMin;sjMax] of
			// width 2*lambda_descr*sigma_key*(nhist+1)/nhist
			float R = (1 + 1 / (float)param.n_hist)*param.lambda_descr*sigma;
			float Rp = 1.414f * R;
			int siMin = std::max(0, (int)(x - Rp + 0.5));
			int sjMin = std::max(0, (int)(y - Rp + 0.5));
			int siMax = std::min((int)(x + Rp + 0.5), w - 1);
			int sjMax = std::min((int)(y + Rp + 0.5), h - 1);
			/// For each pixel inside the patch.
			for (int si = siMin; si < siMax; si++) {
				for (int sj = sjMin; sj < sjMax; sj++) {
					// Compute pixel coordinates (sX,sY) on keypoint's invariant referential.
					float X = si - x;
					float Y = sj - y;
					applyRotation(X, Y, &X, &Y, -keys[ik].theta);
					// Does this sample fall inside the descriptor area ?
					if (std::max(abs(X), abs(Y)) < R) {
						// Compute the gradient orientation (theta) on keypoint referential.
						float dx = gradientX[o][s]->at(si,sj);
						float dy = gradientY[o][s]->at(si, sj);
						float ori = atan2f(dy, dx) - keys[ik].theta;
						//ori = modulus(ori, 2 * M_PI);
						// Compute the gradient magnitude and apply a Gaussian weighing to give less emphasis to distant sample
						float t = param.lambda_descr * sigma;
						float M = hypot(dx, dy) * exp(-(X*X + Y * Y) / (2 * t*t));

						// bin indices, Compute the (tri)linear weightings ...
						float alpha = X / (2 * param.lambda_descr*sigma / param.n_hist) + (param.n_hist - 1.0f) / 2.0f;
						float beta = Y / (2 * param.lambda_descr*sigma / param.n_hist) + (param.n_hist - 1.0f) / 2.0f;
						float gamma = ori / (2 * 3.1415926f)*param.n_ori;
						//    ...and add contributions to respective bins in different histograms.
						// a loop with 1 or two elements
						int i0 = static_cast<int>(floor(alpha));
						int j0 = static_cast<int>(floor(beta));
						for (int i = std::max(0, i0); i <= std::min(i0 + 1, param.n_hist - 1); i++) {
							for (int j = std::max(0, j0); j <= std::min(j0 + 1, param.n_hist - 1); j++) { // looping through all surrounding histograms.

								int k;
								// Contribution to left bin.
								k = ((int)gamma + param.n_ori) % param.n_ori;
								keys[ik].desc[i*param.n_hist*param.n_ori + j * param.n_ori + k] += (1.f - (gamma - floor(gamma)))
									*(1.0f - abs((float)i - alpha))
									*(1.0f - abs((float)j - beta))
									*M;

								// Contribution to right bin.
								k = ((int)gamma + 1 + param.n_ori) % param.n_ori;
								keys[ik].desc[i*param.n_hist*param.n_ori + j * param.n_ori + k] += (1.0f - (floor(gamma) + 1 - gamma))
									*(1.0f - abs((float)i - alpha))
									*(1.0f - abs((float)j - beta))
									*M;


							}
						}
					}
				}
			}

			// Threshold and quantization of the descriptor
			// Normalize
			float l2norm = applyL2Norm(keys[ik].desc, n_descr);
			// Threshold bins
			for (int i = 0; i < n_descr; i++) {
				keys[ik].desc[i] = std::min(keys[ik].desc[i], 0.2f*l2norm);
			}
			// Renormalize
			l2norm = applyL2Norm(keys[ik].desc, n_descr);
			// Quantization
			for (int i = 0; i < n_descr; i++) {
				keys[ik].desc[i] = static_cast<float>(keys[ik].desc[i] * 512.0f / l2norm);
				keys[ik].desc[i] = std::min(keys[ik].desc[i], 255.0f);
			}
		}
	}

	void run(const CImage<float>* vImage) {
		originalImage = new CImage<float>(vImage->ncols, vImage->nrows);
		memcpy(originalImage->data, vImage->data, sizeof(float)*vImage->ncols*vImage->nrows);
		
		init(vImage->ncols,vImage->nrows);
		scalespaceCompute(vImage);
		scalespaceComputeDog();
		extractExtrema();
		
		discardKeysWithLowContrast(0.8f*param.keysThreshold);
		refineKeyPoint();
		discardKeysWithLowContrast(param.keysThreshold);
		discardKeysOnEdge();

		preComputerGradient();
		computeOrientation();
		computeDescriptor();
	}

};

void computeKeypointsDistance(std::vector<std::vector<float>> &distVector
	, const CSIFT* detector1, const CSIFT* detector2) {
	for (size_t index1 = 0; index1 < detector1->keys.size(); index1++)
	{
		for (size_t index2 = 0; index2 < detector2->keys.size(); index2++)
		{
			assert(detector1->keys[0].desc.size() == detector2->keys[0].desc.size());
			float dist = 0.f;
			for (size_t featureIndex = 0; featureIndex < detector1->keys[index1].desc.size(); featureIndex++)
			{
				float t = detector1->keys[index1].desc[featureIndex] 
					- detector2->keys[index2].desc[featureIndex];
				dist += t * t;
			}
			dist = std::sqrt(dist);
			distVector[index1][index2] = dist;
		}
	}
}

std::vector<std::pair<size_t, size_t>> findTheTwoNearestKeys(const std::vector<std::vector<float>> &distVector
	, const CSIFT* detector1, const CSIFT* detector2) {
	std::vector<std::pair<size_t, size_t>> outVector;
	for (size_t keyIndex = 0; keyIndex < detector1->keys.size(); keyIndex++){
		size_t lessIndex = 0;
		size_t lesslessIndex = 0;
		for (size_t distIndex = 0; distIndex < distVector[keyIndex].size(); distIndex++){
			if (distVector[keyIndex][distIndex] < distVector[keyIndex][lesslessIndex]) {
				if (distVector[keyIndex][distIndex] < distVector[keyIndex][lessIndex]) {
					lesslessIndex = lessIndex;
					lessIndex = distIndex;
				}
				else
					lesslessIndex = distIndex;
			}
		}
		outVector.push_back(std::make_pair(lessIndex, lesslessIndex));
	}
	return outVector;
}

std::vector<std::pair<size_t, size_t>> matchSIFT(const CSIFT* detector1, const CSIFT* detector2) {
	size_t num1 = detector1->keys.size();
	size_t num2 = detector2->keys.size();

	std::vector<std::vector<float>> distVector(num1);
	for (size_t i = 0; i < distVector.size(); i++)
		distVector[i].resize(num2, 0.0f);

	std::vector<std::pair<size_t, size_t>> matchIndexs;

	computeKeypointsDistance(distVector, detector1, detector2);
	matchIndexs=findTheTwoNearestKeys(distVector, detector1, detector2);

	std::vector<std::pair<size_t, size_t>> matchResult;
	for (size_t matchItem = 0; matchItem < matchIndexs.size(); matchItem++)
	{
		float t = distVector[matchItem][matchIndexs[matchItem].first] / distVector[matchItem][matchIndexs[matchItem].second];
		if (distVector[matchItem][matchIndexs[matchItem].first]
			/ distVector[matchItem][matchIndexs[matchItem].second] < MATCHTHRESHOLD) {
			matchResult.push_back(std::make_pair(matchItem, matchIndexs[matchItem].first));
		}
		else
			int a = 1;
	}

	return matchResult;
}

void computeFeaturesOpenCV(std::string vImagePath
	, std::vector<cv::KeyPoint>* vKeyPoints, cv::Mat* vDescriptor
	,CImage<float>& vOriginalImage, size_t& vWidth, size_t& vHeight) {
	FREE_IMAGE_FORMAT fifmt = FreeImage_GetFileType(vImagePath.c_str(), 0);
	FIBITMAP *dib = FreeImage_Load(fifmt, vImagePath.c_str(), 0);

	// Convert non-32 bit images
	if (FreeImage_GetBPP(dib) != 32){
		FIBITMAP* hOldImage = dib;
		dib = FreeImage_ConvertTo32Bits(hOldImage);
		FreeImage_Unload(hOldImage);
	}

	FIBITMAP* greyScaleBitmap = FreeImage_ConvertToGreyscale(dib);
	BYTE *pixels = (BYTE*)FreeImage_GetBits(greyScaleBitmap);
	int width = FreeImage_GetWidth(greyScaleBitmap);
	int height = FreeImage_GetHeight(greyScaleBitmap);

	vOriginalImage = CImage<float>(width, height);

	vWidth = width;
	vHeight = height;
	//SIFT in opencv
	cv::Mat cvImage1(cv::Size(width, height), CV_8U, cv::Scalar(0));
	for (int y = 0; y < height; ++y) {
		BYTE* pixelScanline = FreeImage_GetScanLine(greyScaleBitmap, height - y - 1);
		for (int x = 0; x < width; x++) {
			cvImage1.at<uchar>(y, x) 
				= static_cast<unsigned char>(pixelScanline[x]);
			vOriginalImage.at(x, y) 
				= static_cast<float>(pixelScanline[x]);
		}
	}

	cv::Ptr<cv::Feature2D> sift = cv::xfeatures2d::SIFT::create();
	
	sift->detectAndCompute(cvImage1, cv::Mat(), *vKeyPoints, *vDescriptor);

	for (size_t j = 0; j < vKeyPoints->size(); ++j) {
		vKeyPoints->at(j).pt.x /= static_cast<float>(width);
		vKeyPoints->at(j).pt.y /= static_cast<float>(height);
	}

	FreeImage_Unload(dib); 
	FreeImage_Unload(greyScaleBitmap);
;}

void matchFeatureOpenCV(const cv::Mat* vDescriptor1, const cv::Mat* vDescriptor2
	, std::vector<cv::DMatch>& vGoodMatches) {
	cv::FlannBasedMatcher matcher;
	std::vector <std::vector<cv::DMatch>> matchItems;
	matcher.knnMatch(*vDescriptor1, *vDescriptor2, matchItems, 2);

	for (int i = 0; i < vDescriptor1->rows; i++){
		if (matchItems[i][0].distance / matchItems[i][1].distance <= 0.8f){
			vGoodMatches.push_back(matchItems[i][0]);
		}
	}

}

#endif // 
