/*  Copyright Paul Cernea, September 2024.
All Rights Reserved.*/

#ifndef COMPUTATIONAL_BIOLOGY_H
#define COMPUTATIONAL_BIOLOGY_H

#include <cuda_runtime.h>
// __CUDA_RUNTIME_H__ is now a defined macro.

#include <string>
#include <vector>

namespace ComputationalBiology
{
  __host__ __device__ void lcSequence(char* strA, int lenA, char* strB, int lenB, char* longest, int* oLen);
  __host__ __device__ void lcSubString(char* strA, int lenA, char* strB, int lenB, char* longest, int* oLen);

  /**
   *  Because |LCS(A_k, A_l)| >= |LCS(A_1, A_2, ..., A_n)| we can use parallelism here to speed up large computations
   *  by preprocessing a pairwise bound.
   */
  __global__ void pairwiseLcSubString(char* strArrays, int numArrays, int arrLength, char* oStringOut, int* oLcsLengths);
  void lcSubString(const std::vector<std::string>& strArrays, char* longest, int* oLen, int preprocessedLimit);
}

#endif //def COMPUTATIONAL_BIOLOGY_H
