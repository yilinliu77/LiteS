#include "cuda_util.h"

__global__ void VecAdd(float* A, float* B, float* C)
{
	int i = threadIdx.x;
	C[i] = A[i] + B[i];
}

void execute(float* d_A, float* d_B, float* d_C) {
	int threadsPerBlock = 256;
	int blocksPerGrid =
		(64 + threadsPerBlock - 1) / threadsPerBlock;
	VecAdd <<<blocksPerGrid, threadsPerBlock >>> (d_A, d_B, d_C);
}