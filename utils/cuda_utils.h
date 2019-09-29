#ifndef CUDA_UTILS_
#define CUDA_UTILS_

#include <cuda_runtime_api.h>
#include <cuda.h>

void FatalError(const int lineNumber = 0);
void checkCUDA(const int lineNumber, cudaError_t status);

#endif
