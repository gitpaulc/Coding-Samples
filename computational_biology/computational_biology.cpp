/*  Copyright Paul Cernea, September 2024.
All Rights Reserved.*/

#include "computational_biology.h"

#include <iostream>
#include <stdlib.h>

namespace ComputationalBiology
{
  __host__ __device__ char upper(char c)
  {
	if (c == 'g') { return 'G'; }
	if (c == 'a') { return 'A'; }
	if (c == 't') { return 'T'; }
	if (c == 'c') { return 'C'; }
	return c;
  }

  __host__ __device__ void reverse(char* str, int len)
  {
	int lim = (len / 2) + 1;
	if (len == 0) { return; }
	for (int i = 0; i < lim; i++)
	{
	  char temp = str[i];
	  str[i] = str[len - 1 - i];
	  str[len - 1 - i] = temp;
	}
  }

  void lcSequence(char* strA, int lenA, char* strB, int lenB, char* longest, int* oLen)
  {
    int* lcs = (int*)malloc(sizeof(int) * (lenA + 1) * (lenB + 1));
	if (lcs == nullptr) { return; }
	for (int i = 0; i <= lenA; i++)
	{
	  for (int j = 0; j <= lenB; j++)
      {
	    int k = i * (lenB + 1) + j;
		if ((i == 0) || (j == 0))
		{
		  lcs[k] = 0; // Base case.
		  continue;
		}
		if (upper(strA[i - 1]) == upper(strB[j - 1]))
		{
		  int l = (i - 1) * (lenB + 1) + (j - 1);
		  lcs[k] = 1 + lcs[l];
		  continue;
		}
		int kk = (i - 1) * (lenB + 1) + j;
		int ll = i * (lenB + 1) + (j - 1);
		if (lcs[ll] > lcs[kk])
		{
		  lcs[k] = lcs[ll];
		  continue;
		}
		lcs[k] = lcs[kk];
	  }
	}
	if (oLen != nullptr) { *oLen = lcs[lenA * (lenB + 1) + lenB]; }
	int i = lenA; int j = lenB;
	int k = 0;
	while ((i > 0) && (j > 0))
	{
	  if (upper(strA[i - 1]) == upper(strB[j - 1]))
	  {
	    longest[k] = strA[i - 1];
		++k;
	    i--;
		j--;
		continue;
	  }
	  int kk = (i - 1) * (lenB + 1) + j;
	  int ll = i * (lenB + 1) + (j - 1);
	  if (lcs[ll] > lcs[kk])
	  {
	    j--;
		continue;
	  }
	  i--;
	}
	free(lcs);
	reverse(longest, *oLen);
  }

  void lcSubString(char* strA, int lenA, char* strB, int lenB, char* longest, int* oLen)
  {
    int* lcs = (int*)malloc(sizeof(int) * (lenA + 1) * (lenB + 1));
	if (lcs == nullptr) { return; }
	int bestLength = 0;
	int bestIndexA = 0;
	for (int i = 0; i <= lenA; i++)
	{
	  for (int j = 0; j <= lenB; j++)
	  {
	    if ((i == 0) || (j == 0))
	    {
		  continue; // Base case.
		}
		int k = i * (lenB + 1) + j;
		if (upper(strA[i - 1]) == upper(strB[j - 1]))
		{
		  int l = (i - 1) * (lenB + 1) + (j - 1);
		  lcs[k] = 1 + lcs[l];
		  if (lcs[k] <= bestLength) { continue; }
		  bestLength = lcs[k];
		  bestIndexA = i - 1;
		  continue;
		}
		lcs[k] = 0;
	  }
	}
	if (oLen != nullptr) { *oLen = bestLength; }
	free(lcs);
	if (bestLength == 0) { return; }
	for (int i = bestIndexA; i >= 0; i--)
	{
	  longest[bestIndexA - i] = upper(strA[i]);
	}
	reverse(longest, *oLen);
  }

  __global__ void pairwiseLcSubString(char* strArrays, int numArrays, int arrLength,
	  char* oStringOut, int* oLcsLengths)
  {
	int indStrA = blockIdx.y * blockDim.y + threadIdx.y; // row
	int indStrB = blockIdx.x * blockDim.x + threadIdx.x; // col

	if (indStrA >= numArrays) { return; }  // Check boundary conditions.
	if (indStrA < 0) { return; }
	if (indStrB >= numArrays) { return; }
	if (indStrB < 0) { return; }

	int oLen = 1;
	lcSubString(strArrays + indStrA * arrLength, arrLength, strArrays + indStrB * arrLength, arrLength, oStringOut, &oLen);
	oLcsLengths[indStrA * numArrays + indStrB] = oLen;
  }

  void lcSubString(const std::vector<std::string>& strArrays, char* longest, int* oLen, int preprocessedLimit)
  {
	int bestLength = 0;
	int bestIndexA = 0;
	if (oLen == nullptr) { return; }
	if (strArrays.empty()) { *oLen = 0; return; }
	int numArrays = (int)(strArrays.size());
	if (strArrays.size() == 1)
	{
	  *oLen = (int)(strArrays[0].size());
	  for (int i = 0; i < (*oLen); i++)
	  {
		longest[i] = strArrays[0][i];
	  }
	  return;
	}

	std::vector<int> indices(numArrays, 0);
	while (true)
	{
	  bool bWithinBounds = false;
	  for (int i = 0; i < numArrays; ++i)
	  {
	    if (indices[i] < (int)(strArrays[i].size()))
		{
		  bWithinBounds = true;
		  break;
		}
	  }
	  if (!bWithinBounds) { break; }

	  int currentLength = 0;
	  bool bIncreaseLength = true;
	  while (bIncreaseLength)
	  {
		char currentChar = 'A';
		if (currentLength > 0)
		{
		  currentChar = 'A';
		}
		for (int i = 0; i < numArrays; ++i)
		{
		  int indexI = indices[i] + currentLength;
		  if (indexI >= (int)(strArrays[i].size()))
		  {
		    bIncreaseLength = false;
		    break;
		  }
		  if (i == 0)
		  {
			currentChar = strArrays[0][indexI]; continue;
		  }
		  //else
		  if (strArrays[i][indexI] != currentChar)
		  {
			bIncreaseLength = false;
			break;
		  }
		}
		if (!bIncreaseLength) { break; }
		++currentLength;
	  }

	  if (currentLength > bestLength)
	  {
		bestIndexA = indices[0];
		bestLength = currentLength;
		if (preprocessedLimit > 0)
		{
		  if (bestLength >= preprocessedLimit) { break; }
		}
	  }

	  for (int i = numArrays - 1; i >= 0; --i) // Increment multiindex.
	  {
		if (indices[i] < (int)(strArrays[i].size()))
		{
		  ++indices[i];
		  break;
		}
		indices[i] = 0;
	  }
	}

	if (oLen != nullptr) { *oLen = bestLength; }
	if (bestLength == 0) { return; }
	for (int i = 0; i < bestLength; ++i)
	{
	  longest[i] = upper((strArrays[0])[bestIndexA + i]);
	}
  }

  static void randomBioStr(char* str, int NN)
  {
	for (int i = 0; i < NN; ++i)
	{
	  int letter = rand() % 4;
	  if (letter == 0) { str[i] = 'G'; continue; }
	  if (letter == 1) { str[i] = 'A'; continue; }
	  if (letter == 2) { str[i] = 'T'; continue; }
	  // else if (letter == 3)
	    str[i] = 'C'; // continue;
	}
  }

  static void randomBioStr(std::vector<std::string>& strs, int NN)
  {
    for (auto& str : strs)
	{
	  str.resize(NN);
      for (int i = 0; i < NN; ++i)
	  {
	    int letter = rand() % 4;
		if (letter == 0) { str[i] = 'G'; continue; }
		if (letter == 1) { str[i] = 'A'; continue; }
		if (letter == 2) { str[i] = 'T'; continue; }
		// else if (letter == 3)
		str[i] = 'C'; // continue;
	  }
	}
  }

  static void print(const char* str, int NN)
  {
    for (int i = 0; i < NN; ++i) { std::cout << str[i]; }
  }
}

int main(int argc, char* argv[]) // argv[argc] guaranteed to be a null pointer.
{
  srand((unsigned)time(NULL)); // Seed random number generator.
  char inputChar = 'o';
  while (true)
  {
	int NN = 50;
	std::cout << "\n\nWhich algorithm?";
	std::cout << "\n(A) Longest Common Subsequence.";
	std::cout << "\n(B) Longest Common Substring.";
	std::cout << "\n(Q) Quit.";
	std::cout << "\n\n-->  ";
	std::cin >> inputChar;
	if ((inputChar == 'Q') || (inputChar == 'q')) { break; }
	char* strA = (char*)malloc(sizeof(char) * NN);
	char* strB = (char*)malloc(sizeof(char) * NN);
	char* oData = (char*)malloc(sizeof(char) * NN);
	if ((strA == nullptr) || (strB == nullptr) || (oData == nullptr))
	{
	  std::cout << "\nCould not allocate memory!";
	  continue;
	}
	if ((inputChar != 'B') && (inputChar != 'b'))
	{
	  ComputationalBiology::randomBioStr(strA, NN);
	  ComputationalBiology::randomBioStr(strB, NN);
	  std::cout << "\n\nFirst string:\n";
	  ComputationalBiology::print(strA, NN);
	  std::cout << "\nSecond string:\n";
	  ComputationalBiology::print(strB, NN);
	}

	for (int i = 0; i < NN; ++i) { oData[i] = 0; }
	int oLen = 0;

	if ((inputChar == 'A') || (inputChar == 'a'))
	{
	  ComputationalBiology::lcSequence(strA, NN, strB, NN, oData, &oLen);
	  std::cout << "\nLongest common subsequence:\n";
	  ComputationalBiology::print(oData, oLen);
	}
	else
	{
	  int numStrings = 0;
	  std::cout << "\nHow many strings?";
	  std::cout << "\n\n-->  ";
	  std::cin >> numStrings;
	  if (numStrings <= 0) { numStrings = 1; }
	  std::cout << "\n" << numStrings << " strings.";
	  std::cout << "\nHow many nucleobases (G, A, T, C) per string?";
	  std::cout << "\n\n-->  ";
	  std::cin >> NN;
	  if (NN <= 0) { NN = 50; }
	  std::cout << "\nStrings have " << NN << " nucleobases each.";

	  std::vector<std::string> strs(numStrings);
	  ComputationalBiology::randomBioStr(strs, NN);
	  for (int i = 0; i < numStrings; ++i)
	  {
        std::cout << "\nString " << i + 1 << ":\n";
        ComputationalBiology::print(strs[i].data(), NN);
	  }
	  int preprocessingLim = -1;
	  // CUDA routine:
	  {
	    char* d_Strings = nullptr;
		int* d_LcsLengths = nullptr;
		char* d_StringOut = nullptr;

		cudaMalloc((void**)&(d_Strings), sizeof(char) * numStrings * NN); // Allocate memory on the GPU.
		cudaMalloc((void**)&(d_LcsLengths), sizeof(int) * numStrings * numStrings);
		cudaMalloc((void**)&(d_StringOut), sizeof(char) * NN);
		std::vector<char> hostStrings(numStrings * NN, 0);
		std::vector<int> arrLengthsOut(numStrings * numStrings);

		for (int i = 0; i < numStrings; ++i)
		{
		  for (int j = 0; j < (int)(strs[i].size()); ++j) { hostStrings[i * NN + j] = strs[i][j]; }
		}
		cudaMemcpy(d_Strings, hostStrings.data(), sizeof(char) * numStrings * NN, cudaMemcpyHostToDevice);

		int threadsPerBlock = 16;
		dim3 blockSize(threadsPerBlock, threadsPerBlock); // Product must be <= 1024
		int numBlocks = numStrings / threadsPerBlock + 1;
		dim3 blocksPerGrid(numBlocks, numBlocks);
		ComputationalBiology::pairwiseLcSubString<<<blocksPerGrid, blockSize>>>(d_Strings, numStrings, NN, d_StringOut, d_LcsLengths);
		auto lastError = cudaGetLastError();

		cudaDeviceSynchronize();
		cudaMemcpy(arrLengthsOut.data(), d_LcsLengths, sizeof(int) * numStrings * numStrings, cudaMemcpyDeviceToHost);

		cudaFree(&d_Strings); // Free GPU memory.
		cudaFree(&d_LcsLengths);
		cudaFree(&d_StringOut);

		for (int i = 0; i < numStrings; ++i)
		{
		  for (int j = i + 1; j < numStrings; ++j)
		  {
			int current = arrLengthsOut[i * numStrings + j];
			if ((i == 0) && (j == 1)) { preprocessingLim = current; continue; }
			if (current < preprocessingLim) { preprocessingLim = current; }
		  }
		}
	  }
	  ComputationalBiology::lcSubString(strs, oData, &oLen, preprocessingLim);
	  std::cout << "\nLongest common substring:\n";
	  if (oLen == 0) { std::cout << "Empty."; }
	  ComputationalBiology::print(oData, oLen);
	}
	free(strA);
	free(strB);
	free(oData);
	std::cout << "\n\nPress any key to continue, or (Q) to quit.";
	std::cout << "\n-->  ";
	char dummyChar = 'o';
	std::cin >> dummyChar;
	if ((inputChar == 'Q') || (inputChar == 'q')) { break; }
  }
  return 0;
}
