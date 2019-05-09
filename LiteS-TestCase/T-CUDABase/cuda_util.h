#pragma once

#include <cuda_runtime.h>

#include <cuda.h>
#include<device_launch_parameters.h>

extern __global__ void VecAdd(float* A, float* B, float* C);

extern void execute(float* d_A, float* d_B, float* d_C);