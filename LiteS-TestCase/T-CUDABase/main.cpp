#include <iostream>
#include "cuda_util.h"

int main() {
	int N = 64;
	size_t size = N * sizeof(float);

	// Allocate input vectors h_A and h_B in host memory
	float* h_A = (float*)malloc(size);
	float* h_B = (float*)malloc(size);

	// Initialize input vectors


	// Allocate vectors in device memory
	float* d_A;
	cudaMalloc(&d_A, size);
	float* d_B;
	cudaMalloc(&d_B, size);
	float* d_C;
	cudaMalloc(&d_C, size);

	// Copy vectors from host memory to device memory
	cudaMemcpy(d_A, h_A, size, cudaMemcpyHostToDevice);
	cudaMemcpy(d_B, h_B, size, cudaMemcpyHostToDevice);

	// Invoke kernel
	execute(d_A, d_B, d_C);
	

	return 0;
}