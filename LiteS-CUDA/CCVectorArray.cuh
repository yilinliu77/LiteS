#ifndef CCVECTOR_ARRAY_H
#define CCVECTOR_ARRAY_H

#include <cuda_runtime.h>
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>

#include "CCDataStructure.h"

namespace CCDataStructure {

template <typename T>
class CCVectorArray {
 public:
  struct Data {
    unsigned int* numRowsPtr;
    size_t maxRows;
    size_t maxCols;
    T* dataPtr;
    size_t pitch;
  };

  Data data;

  CCVectorArray() : data({nullptr, 0, 0,  nullptr, 0}){};

  CCVectorArray(size_t vCols, size_t vRows)
      : data({nullptr, 0, 0,  nullptr, 0}) {
    data.maxCols = vCols;
    data.maxRows = vRows;

    CUDACHECKERROR(cudaMalloc(&data.numRowsPtr, vCols * sizeof(unsigned int)));
    CUDACHECKERROR(
        cudaMemset(data.numRowsPtr, 0, vCols * sizeof(unsigned int)));
    CUDACHECKERROR(
        cudaMallocPitch(&data.dataPtr, &data.pitch, vCols * sizeof(T), vRows));
  }

  
};
}  // namespace CCDataStructure

#endif