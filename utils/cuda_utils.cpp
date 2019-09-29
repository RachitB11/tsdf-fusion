#include "cuda_utils.h"
#include <iostream>

void FatalError(const int lineNumber) {
  std::cerr << "FatalError";
  if (lineNumber != 0) std::cerr << " at LINE " << lineNumber;
  std::cerr << ". Program Terminated." << std::endl;
  cudaDeviceReset();
  exit(EXIT_FAILURE);
}

void checkCUDA(const int lineNumber, cudaError_t status) {
  if (status != cudaSuccess) {
    std::cerr << "CUDA failure at LINE " << lineNumber << ": " << status << std::endl;
    FatalError();
  }
}
