
#include "includes.h"
#include "point_cloud.h"
#include "gl_callbacks.h"

#ifdef _WIN32
#include <algorithm>
#endif
#include <map>

#define USE_MULTI_THREADING

#ifdef USE_MULTI_THREADING
#include <mutex>
#include <thread>
#endif // USE_MULTI_THREADING

namespace ComputationalGeometry
{
  static int gWindowWidth = 1024;
  static int gWindowHeight = 1024;

  static point2d gScreenMin(-1, -1);
  static point2d gScreenMax(1, 1);

#ifdef USE_MULTI_THREADING
  static std::mutex gMutex;
#endif // USE_MULTI_THREADING

  int& numRandomPoints()
  {
    static int numberOfRandomPoints = 50;
    return numberOfRandomPoints;
  }

  void SetWindowWidthHeight(int ww, int hh)
  {
    ComputationalGeometry::gWindowWidth = ww;
    if (hh < 0) { hh = ww; }
    ComputationalGeometry::gWindowHeight = hh;
  }

  void GetWindowWidthHeight(int& ww, int& hh)
  {
    ww = gWindowWidth;
    hh = gWindowHeight;
  }

  double threshold() { return 1.0e-9; }

  static double safeSqrt(const double& xx)
  {
    if (xx <= threshold()) { return 0; }
    return sqrt(xx);
  }

  void ComputationalGeometry::MergeConvex(point2d* iConvexA, int iConvexASize, point2d* iConvexB, int iConvexBSize, point2d* oMerged, int* oMergedSize)
  {
    if (iConvexASize == 0)
    {
      for (int ii = 0; ii < iConvexBSize; ++ii) { oMerged[ii] = iConvexB[ii]; }
      *oMergedSize = iConvexBSize;
      return;
    }
    if (iConvexBSize == 0)
    {
      for (int ii = 0; ii < iConvexASize; ++ii) { oMerged[ii] = iConvexA[ii]; }
      *oMergedSize = iConvexASize;
      return;
    }
    int minIndexA = 0;
    int minIndexA_Y = 0;
    int maxIndexA_Y = 0;
    int minIndexB = 0;
    int minIndexB_Y = 0;
    int maxIndexB_Y = 0;
    for (int ii = 0; ii < iConvexASize; ++ii)
    {
      if (iConvexA[ii] < iConvexA[minIndexA]) { minIndexA = ii; }
      if (iConvexA[ii].y < iConvexA[minIndexA_Y].y) { minIndexA_Y = ii; }
      if (iConvexA[ii].y > iConvexA[maxIndexA_Y].y) { maxIndexA_Y = ii; }
    }
    for (int ii = 0; ii < iConvexBSize; ++ii)
    {
      if (iConvexB[ii] < iConvexB[minIndexB]) { minIndexB = ii; }
      if (iConvexB[ii].y < iConvexB[minIndexB_Y].y) { minIndexB_Y = ii; }
      if (iConvexB[ii].y > iConvexB[maxIndexB_Y].y) { maxIndexB_Y = ii; }
    }

    // TODO: Make this run in linear time.
    // For now the hull is so small that in practice the time O(h * log(h)) is linear O(h).
    {
      PointCloud pc(iConvexASize + iConvexBSize);
      for (int ii = 0; ii < iConvexASize; ++ii)
      {
        pc.setPoint(ii, iConvexA[ii]);
      }
      for (int ii = 0; ii < iConvexBSize; ++ii)
      {
        pc.setPoint(iConvexASize + ii, iConvexB[ii]);
      }
      pc.computeConvexHull();
      *oMergedSize = (int)(pc.ConvexHull().size());
      for (int ii = 0; ii < *oMergedSize; ++ii)
      {
        oMerged[ii] = pc.ConvexHull()[ii];
      }
    }
  }

  void ComputationalGeometry::MergeConvex3d(point3d* iConvexA, int iConvexASize, point3d* iConvexB, int iConvexBSize, point3d* oMerged, int* oMergedSize)
  {
    if (iConvexASize == 0)
    {
      for (int ii = 0; ii < iConvexBSize; ++ii) { oMerged[ii] = iConvexB[ii]; }
      *oMergedSize = iConvexBSize;
      return;
    }
    if (iConvexBSize == 0)
    {
      for (int ii = 0; ii < iConvexASize; ++ii) { oMerged[ii] = iConvexA[ii]; }
      *oMergedSize = iConvexASize;
      return;
    }
    // TODO: Make this run in linear time.
    // For now the |hull| is typically much less than |cloud|
    std::vector<point3d> mergedArray(iConvexASize + iConvexBSize);
    for (int ii = 0; ii < iConvexASize; ++ii)
    {
      mergedArray[ii] = iConvexA[ii];
    }
    for (int ii = 0; ii < iConvexBSize; ++ii)
    {
      mergedArray[iConvexASize + ii] = iConvexB[ii];
    }
    std::vector<int> isInHull(iConvexASize + iConvexBSize);
    computeConvexHull3d(mergedArray.data(), (int)mergedArray.size(), isInHull.data());
    int hullSize = 0;
    for (int ii = 0; ii < (int)(mergedArray.size()); ++ii)
    {
      if (isInHull[ii] == 0) { continue; }
      oMerged[hullSize] = mergedArray[ii];
      ++hullSize;
    }
    *oMergedSize = hullSize;
  }

  std::vector<point2d> ComputationalGeometry::ConvexHullDivideAndConquer(std::vector<point2d>& iCloud)
  {
    const int convexHullSmall = 25;
    if (iCloud.size() == 0) { return iCloud; }
    std::sort(iCloud.begin(), iCloud.end()); // Sort by x-coordinate first and foremost;
    if (iCloud.size() <= 3) { return iCloud; }
    int cloudSize = (int)iCloud.size();

    std::vector<int> partitionsCloud;
    std::vector<int> partitionsHull;
    int partitionsLength = 1;
    int stackDepth = 0;
    {
      int initialSize = 1;
      for (int ii = 0; ii < 1000; ++ii)
      {
        initialSize *= 2;
        if (initialSize >= cloudSize) { break; }
      }
      int smallSize = initialSize;
      while ((smallSize > convexHullSmall) || (stackDepth == 0))
      {
        smallSize = smallSize / 2;
        partitionsLength *= 2;
        ++stackDepth;
      }

      partitionsCloud = std::vector<int>(partitionsLength, smallSize);
      partitionsHull.resize(partitionsLength);
    }

    std::vector<point2d> cloudArr = iCloud;
    std::vector<point2d> hull(cloudSize);

    // Try to ensure partition line does not occur along a common x-coordinate.
    {
      int startIndex = 1;
      for (int pp = startIndex; pp < partitionsLength; ++pp)
      {
        int ii = partitionsCloud[pp - 1];
        int diff = 0;
        for (int jj = 0; jj < partitionsCloud[pp] - startIndex; ++jj)
        {
          double deltaX = (cloudArr[ii].x - cloudArr[jj].x);
          if (deltaX * deltaX <= threshold()) { ++diff; continue; }
          break;
        }
        partitionsCloud[pp - 1] = partitionsCloud[pp - 1] + diff;
        partitionsCloud[pp] = partitionsCloud[pp] - diff;
      }
    }
    int hullSize = 0;
#ifdef __CUDA_RUNTIME_H__
    {
        int maxPartitionSize = 0; // Column count.
        // Initialize host arrays.
        for (int pp = 0; pp < partitionsLength; ++pp) // Row count.
        {
          if (partitionsCloud[pp] > maxPartitionSize) { maxPartitionSize = partitionsCloud[pp]; }
        }
        std::vector<point2d> hostArrays(partitionsLength * maxPartitionSize);
        
        int startIndex = 0;
        for (int pp = 0; pp < partitionsLength; ++pp)
        {
          for (int ii = 0; ii < partitionsCloud[pp]; ++ii)
          {
            if ((startIndex + ii) >= (int)(cloudArr.size())) { break; }
            hostArrays[pp * maxPartitionSize + ii] = cloudArr[startIndex + ii];
          }
          startIndex += partitionsCloud[pp];
        }

        std::vector<int> oIndicatorArrays(partitionsLength * maxPartitionSize, 1);
        // Initialize device arrays.
        point2d* d_PointArrays;
        int* d_ArraySizes;
        int* d_oIndicatorArrays;
        // Allocate memory on the GPU.
        cudaMalloc((void**)&(d_PointArrays), sizeof(point2d) * partitionsLength * maxPartitionSize);
        cudaMalloc((void**)&(d_ArraySizes), sizeof(int) * partitionsLength);
        cudaMalloc((void**)&(d_oIndicatorArrays), sizeof(int) * partitionsLength * maxPartitionSize);
        cudaMemcpy(d_PointArrays, hostArrays.data(), sizeof(point2d) * partitionsLength * maxPartitionSize, cudaMemcpyHostToDevice);
        cudaMemcpy(d_oIndicatorArrays, oIndicatorArrays.data(), sizeof(int) * partitionsLength * maxPartitionSize, cudaMemcpyHostToDevice);
        cudaMemcpy(d_ArraySizes, partitionsCloud.data(), sizeof(int) * partitionsLength, cudaMemcpyHostToDevice);

        int threadsPerBlock = 32;
        dim3 blockSize(threadsPerBlock, threadsPerBlock); // Product must be <= 1024
        int numBlocks = partitionsLength * maxPartitionSize / threadsPerBlock + 1;
        dim3 blocksPerGrid(numBlocks, numBlocks);
        ComputeConvexHulls<<<blocksPerGrid, blockSize>>> (d_PointArrays, d_ArraySizes,
          maxPartitionSize, partitionsLength, d_oIndicatorArrays);
        cudaDeviceSynchronize();
        auto lastError = cudaGetLastError();

        cudaMemcpy(oIndicatorArrays.data(), d_oIndicatorArrays, sizeof(int) * partitionsLength * maxPartitionSize, cudaMemcpyDeviceToHost);
        lastError = cudaGetLastError();

        startIndex = 0;
        for (int pp = 0; pp < partitionsLength; ++pp)
        {
          int currentHullSize = 0;
          for (int ii = 0; ii < partitionsCloud[pp]; ++ii)
          {
            if ((startIndex + ii) >= (int)(cloudArr.size())) { break; }
            auto current = cloudArr[startIndex + ii];
            if (oIndicatorArrays[pp * maxPartitionSize + ii] == 0) { continue; }
            hull[currentHullSize + hullSize] = current;
            ++currentHullSize;
          }
          startIndex += partitionsCloud[pp];
          partitionsHull[pp] = currentHullSize;
          hullSize += currentHullSize;
        }

        cudaFree(&d_PointArrays); // Free GPU memory.
        cudaFree(&d_ArraySizes); // Free GPU memory.
        cudaFree(&d_oIndicatorArrays); // Free GPU memory.
    }
#else
//#define TESTING_CUDA
#ifdef TESTING_CUDA
    {
      int startIndex = 0;
      for (int pp = 0; pp < partitionsLength; ++pp)
      {
        int tempSize = partitionsCloud[pp];
        std::vector<point2d> cloudArrCuda(tempSize);
        for (int ii = 0; ii < tempSize; ++ii)
        {
          cloudArrCuda[ii] = cloudArr[startIndex + ii];
        }
        startIndex += tempSize;
        int currentHullSize = 0;
        std::vector<int> bIsIn(tempSize, 1);
        computeConvexHullCuda(cloudArrCuda.data(), tempSize, bIsIn.data());
        for (int ii = 0; ii < tempSize; ++ii)
        {
          if (bIsIn[ii] == 0) { continue; }
          hull[currentHullSize + hullSize] = cloudArrCuda[ii];
          ++currentHullSize;
        }
        partitionsHull[pp] = currentHullSize;
        hullSize += currentHullSize;
      }
    }
#else
    {
      int startIndex = 0;
      for (int ii = 0; ii < partitionsLength; ++ii)
      {
        int tempSize = partitionsCloud[ii];
        PointCloud pc(cloudArr.data() + startIndex, tempSize);
        startIndex += tempSize;
        pc.computeConvexHull();
        const int currentHullSize = (int)(pc.ConvexHull().size());
        partitionsHull[ii] = currentHullSize;
        for (int jj = 0; jj < currentHullSize; ++jj)
        {
          if (jj + hullSize >= cloudSize) { break; }
          hull[jj + hullSize] = pc.ConvexHull()[jj];
        }
        hullSize += currentHullSize;
      }
    }
#endif // def TESTING_CUDA
#endif // def __CUDA_RUNTIME_H__
    cloudSize = hullSize;
    for (int ii = 0; ii < cloudSize; ++ii)
    {
      cloudArr[ii] = hull[ii];
    }
    for (int ii = 0; ii < partitionsLength; ++ii)
    {
      partitionsCloud[ii] = partitionsHull[ii];
    }

    for (;stackDepth > 0; --stackDepth)
    {
      int hullPartitionsLength = 0;
      int increaseBy = 2;
      hullSize = 0;
      int indA = 0;
      for (int ii = 0; ii < partitionsLength; ii = ii + increaseBy)
      {
        const int arrSizeA = partitionsCloud[ii];
        int indB = indA + arrSizeA;
        const int arrSizeB = partitionsCloud[ii + 1];
        int hullSizeOut = 0;
        MergeConvex(cloudArr.data() + indA, arrSizeA, cloudArr.data() + indB, arrSizeB,
                    hull.data() + hullSize, &hullSizeOut);
        indA = indB + arrSizeB;
        partitionsHull[hullPartitionsLength] = hullSizeOut;
        hullSize += hullSizeOut;
        ++hullPartitionsLength;
      }
      cloudSize = hullSize;
      for (int ii = 0; ii < cloudSize; ++ii)
      {
        cloudArr[ii] = hull[ii];
      }
      partitionsLength = hullPartitionsLength;
      for (int ii = 0; ii < partitionsLength; ++ii)
      {
        partitionsCloud[ii] = partitionsHull[ii];
      }
    }

    hull.resize(hullSize);
    return hull;
  }

  std::vector<point3d> ComputationalGeometry::ConvexHull3d(std::vector<point3d>& iCloud)
  {
    const int convexHullSmall = 25;
    if (iCloud.size() == 0) { return iCloud; }
    std::sort(iCloud.begin(), iCloud.end()); // Sort by x-coordinate first and foremost, then y.
    if (iCloud.size() <= 3) { return iCloud; }
    int cloudSize = (int)iCloud.size();

    std::vector<int> partitionsCloud;
    std::vector<int> partitionsHull;
    int partitionsLength = 1;
    int stackDepth = 0;
    {
      int initialSize = 1;
      for (int ii = 0; ii < 1000; ++ii)
      {
        initialSize *= 2;
        if (initialSize >= cloudSize) { break; }
      }
      int smallSize = initialSize;
      while ((smallSize > convexHullSmall) || (stackDepth == 0))
      {
        smallSize = smallSize / 2;
        partitionsLength *= 2;
        ++stackDepth;
      }
      partitionsCloud = std::vector<int>(partitionsLength, smallSize);
      partitionsHull.resize(partitionsLength);
    }

    auto cloudArr = iCloud;
    std::vector<point3d> hull(cloudSize);

    // Try to ensure partition line does not occur along a common x or y-coordinate.
    {
      int startIndex = 1;
      for (int pp = startIndex; pp < partitionsLength; ++pp)
      {
        int ii = partitionsCloud[pp - 1];
        int diff = 0;
        for (int jj = 0; jj < partitionsCloud[pp] - startIndex; ++jj)
        {
          double deltaX = (cloudArr[ii].x - cloudArr[jj].x);
          if (deltaX * deltaX <= threshold()) { ++diff; continue; }
          double deltaY = (cloudArr[ii].y - cloudArr[jj].y);
          if (deltaY * deltaY <= threshold()) { ++diff; continue; }
          break;
        }
        partitionsCloud[pp - 1] = partitionsCloud[pp - 1] + diff;
        partitionsCloud[pp] = partitionsCloud[pp] - diff;
      }
    }
    int hullSize = 0;
    {
      int startIndex = 0;
      for (int pp = 0; pp < partitionsLength; ++pp)
      {
        int tempSize = partitionsCloud[pp];
        std::vector<point3d> cloudArrCuda(tempSize);
        for (int ii = 0; ii < tempSize; ++ii)
        {
          cloudArrCuda[ii] = cloudArr[startIndex + ii];
        }
        startIndex += tempSize;
        int currentHullSize = 0;
        std::vector<int> bIsIn(tempSize, 1);
        computeConvexHull3d(cloudArrCuda.data(), tempSize, bIsIn.data());
        for (int ii = 0; ii < tempSize; ++ii)
        {
          if (bIsIn[ii] == 0) { continue; }
          hull[currentHullSize + hullSize] = cloudArrCuda[ii];
          ++currentHullSize;
        }
        partitionsHull[pp] = currentHullSize;
        hullSize += currentHullSize;
      }
    }
    cloudSize = hullSize;
    for (int ii = 0; ii < cloudSize; ++ii)
    {
      cloudArr[ii] = hull[ii];
    }
    for (int ii = 0; ii < partitionsLength; ++ii)
    {
      partitionsCloud[ii] = partitionsHull[ii];
    }

    for (; stackDepth > 0; --stackDepth)
    {
      int hullPartitionsLength = 0;
      int increaseBy = 2;
      hullSize = 0;
      int indA = 0;
      for (int ii = 0; ii < partitionsLength; ii = ii + increaseBy)
      {
        const int arrSizeA = partitionsCloud[ii];
        int indB = indA + arrSizeA;
        const int arrSizeB = partitionsCloud[ii + 1];
        int hullSizeOut = 0;
        MergeConvex3d(cloudArr.data() + indA, arrSizeA, cloudArr.data() + indB, arrSizeB,
          hull.data() + hullSize, &hullSizeOut);
        indA = indB + arrSizeB;
        partitionsHull[hullPartitionsLength] = hullSizeOut;
        hullSize += hullSizeOut;
        ++hullPartitionsLength;
      }
      cloudSize = hullSize;
      for (int ii = 0; ii < cloudSize; ++ii)
      {
        cloudArr[ii] = hull[ii];
      }
      partitionsLength = hullPartitionsLength;
      for (int ii = 0; ii < partitionsLength; ++ii)
      {
        partitionsCloud[ii] = partitionsHull[ii];
      }
    }

    hull.resize(hullSize);
    return hull;
  }

  __global__ void ComputationalGeometry::CenterOfMass(point2d* aa, int arrSize, point2d* bb)
  {
#ifdef __CUDA_RUNTIME_H__
    int ii = blockIdx.x * blockDim.x + threadIdx.x;
    double inv = 1.0 / (double)arrSize;
    if (ii < arrSize)
    {
      bb[ii].x = aa[ii].x * inv;
      bb[ii].y = aa[ii].y * inv;
    }
#endif // def __CUDA_RUNTIME_H__
  }

  __host__ __device__ void computeConvexHullCuda(point2d* iPointArray, int iNumPoints, int* isInHull)
  {
#ifdef __CUDA_RUNTIME_H__
    for (int ii = 0; ii < iNumPoints; ++ii)
    {
      for (int jj = 0; jj < iNumPoints; ++jj)
      {
        if (iPointArray[ii].sqDistance(iPointArray[jj]) <= threshold()) { continue; }
        for (int kk = 0; kk < iNumPoints; ++kk)
        {
          if (iPointArray[ii].sqDistance(iPointArray[kk]) <= threshold()) { continue; }
          if (iPointArray[kk].sqDistance(iPointArray[jj]) <= threshold()) { continue; }
          Triangle2d face(iPointArray[ii], iPointArray[jj], iPointArray[kk]);
          for (int ll = 0; ll < iNumPoints; ++ll)
          {
            auto pointInterior = face.pointIsInterior(iPointArray[ll]);
            if ((pointInterior != 0) && (pointInterior != 3))
            {
              isInHull[ll] = 0;
            }
          }
        }
      }
    }
#endif // def __CUDA_RUNTIME_H__
  }

  __global__ void ComputeConvexHulls(point2d* pointArrays, int* arraySizes, int maxArrSize, int numArrays, int* oIndicatorArrays)
  {
#ifdef __CUDA_RUNTIME_H__
    int jj = blockIdx.y * blockDim.y + threadIdx.y; // row
    int ii = blockIdx.x * blockDim.x + threadIdx.x; // col
    if (jj >= numArrays) { return; }  // Check boundary conditions.
    if (jj < 0) { return; }
    computeConvexHullCuda(pointArrays + jj * maxArrSize, arraySizes[jj], oIndicatorArrays + jj * maxArrSize);
#endif // def __CUDA_RUNTIME_H__
  }

  __host__ __device__ void computeConvexHull3d(point3d* iPointArray, int iNumPoints, int* isInHull)
  {
#ifdef __CUDA_RUNTIME_H__
    for (int ii = 0; ii < iNumPoints; ++ii)
    {
      for (int jj = 0; jj < iNumPoints; ++jj)
      {
        if (iPointArray[ii].sqDistance(iPointArray[jj]) <= threshold()) { continue; }
        for (int kk = 0; kk < iNumPoints; ++kk)
        {
          if (iPointArray[ii].sqDistance(iPointArray[kk]) <= threshold()) { continue; }
          if (iPointArray[kk].sqDistance(iPointArray[jj]) <= threshold()) { continue; }
          for (int ll = 0; ll < iNumPoints; ++ll)
          {
            if (iPointArray[ii].sqDistance(iPointArray[ll]) <= threshold()) { continue; }
            if (iPointArray[ll].sqDistance(iPointArray[jj]) <= threshold()) { continue; }
            if (iPointArray[kk].sqDistance(iPointArray[ll]) <= threshold()) { continue; }
            Tetrahedron3d face(iPointArray[ii], iPointArray[jj], iPointArray[kk], iPointArray[ll]);
            for (int uu = 0; uu < iNumPoints; ++uu)
            {
              auto pointInterior = face.pointIsInterior(iPointArray[uu]);
              if ((pointInterior != 0) && (pointInterior != 4))
              {
                isInHull[uu] = 0;
              }
            }
          }
        }
      }
    }
#endif // def __CUDA_RUNTIME_H__
  }

  __global__ void ComputeConvexHulls3d(point3d* pointArrays, int* arraySizes, int maxArrSize, int numArrays, int* oIndicatorArrays)
  {
#ifdef __CUDA_RUNTIME_H__
      int jj = blockIdx.y * blockDim.y + threadIdx.y; // row
      int ii = blockIdx.x * blockDim.x + threadIdx.x; // col
      if (jj >= numArrays) { return; }  // Check boundary conditions.
      if (jj < 0) { return; }
      computeConvexHull3d(pointArrays + jj * maxArrSize, arraySizes[jj], oIndicatorArrays + jj * maxArrSize);
#endif // def __CUDA_RUNTIME_H__
  }

  class PointCloud::Impl
  {
  public:
    PointCloud* pCloud = nullptr;

    bool bConvexHullOn = true;
    bool bDelaunayOn = false;
    bool bNearestNeighborOn = false;
    bool bPointsVisible = true;
    bool bTrianglesModeOn = false;
    bool bVoronoiOn = false;

    std::vector<point2d> hull;
    std::vector<point2d> pointArray;
    std::vector<Triangle2d> triangulation;
    std::vector<Triangle2d> delaunay;
    std::vector<Edge2d> nearestNeighbor;
    std::vector<Edge2d> voronoi;

    Impl(PointCloud* pParent) : pCloud(pParent) {}
    /**
     * This is O(n^2 log(n)) where n is the number of vertices.
     * For every vertex, iterate over the number of faces a constant number of times.
     * The number of faces in any triangulation is 2n - size(hull) - 2 which is O(n).
     * So iterating over the faces is O(n log(n))
     * This gives a total of O(n^2 log(n)).
     */
    void naiveTriangulate(std::vector<Triangle2d>& ioTriangulation, std::vector<point2d>& ioPointArray, std::vector<point2d>& ioHull);
    void generateRandomPoints();
    /**
     * Divide by picking a best center point and drawing three best segments to a vertex
     * in the convex hull. Each subregion is the intersection of a triangle with the hull, and so
     * it must be convex.
     */
    void dividePointsInto3ConvexClouds(const std::vector<point2d>& iPointArray, std::vector<point2d>& oPointArray1, std::vector<point2d>& oPointArray2, std::vector<point2d>& oPointArray3);
    void computeHullDivideConquer();
    static const int DelaunayMaxForNaive = 100;
    enum class DelaunayMode
    {
      Naive = 0,
      DivideAndConquer = 1
    };
    void computeDelaunay(DelaunayMode delaunayMode,
                         std::vector<point2d>& ioPointArray,
                         std::vector<Triangle2d>& ioTriangulation,
                         std::vector<point2d>& ioHull,
                         std::vector<Triangle2d>& ioDelaunay);
#ifdef USE_MULTI_THREADING
    std::vector<std::vector<Triangle2d> > mPartialTriangulations;
    std::vector<std::vector<point2d> > mSmallClouds;
    void computeDelaunayThreadFunc(int threadIndex);
#endif // USE_MULTI_THREADING
    void computeNearestNeighbor();
    std::vector<Edge2d> constructVoronoiRays(const point2d& site, const std::vector<point2d>& localDelaunayVertices);
    void computeVoronoi2or3points();
    void computeVoronoi();
      
    /** \brief O(n log(n)) Convex hull implementation. Graham scan for 2d points. */
    void computeConvexHull(const std::vector<point2d>& iPointArray, std::vector<point2d>& ioHull);
    template <class Container> static double naiveMinSqDistance(Container& A, Container& B, point3d& A_min, point3d& B_min);
    template <class Container> static double naiveMinSqDistance(Container& arr, point3d& min_1, point3d& min_2);
    template <class Container> static double minSqDistanceHelper(Container& arr, point2d& min_1, point2d& min_2);
    template <class Container> static double minSqDistance(Container& arr, point2d& min_1, point2d& min_2);
  };

  PointCloud::PointCloud() : pImpl(std::make_unique<PointCloud::Impl>(this))
  {
    // unique_ptr requires C++ 11.
    // make_unique requires C++ 14.
  }

  PointCloud::PointCloud(int iNumPoints) : pImpl(std::make_unique<PointCloud::Impl>(this))
  {
    if (pImpl != nullptr) // Of course this should never happen.
    {
      pImpl->pointArray.resize(iNumPoints);
    }
  }

  PointCloud::PointCloud(point2d* iPoints, int iNumPoints) : pImpl(std::make_unique<PointCloud::Impl>(this))
  {
    if (pImpl != nullptr) // Of course this should never happen.
    {
      pImpl->pointArray.resize(iNumPoints);
      for (int ii = 0; ii < iNumPoints; ++ii)
      {
        pImpl->pointArray[ii] = iPoints[ii];
      }
    }
  }

  const std::vector<point2d>& PointCloud::PointArray() const
  {
    if (pImpl == nullptr) // Of course this should never happen.
    {
      static std::vector<point2d> dummy;
      return dummy;
    }
    return pImpl->pointArray;
  }

  const std::vector<point2d>& PointCloud::ConvexHull() const
  {
    if (pImpl == nullptr)
    {
      static std::vector<point2d> dummy;
      return dummy;
    }
    return pImpl->hull;
  }

  const std::vector<Edge2d>& PointCloud::NearestNeighbor() const
  {
    if (pImpl == nullptr)
    {
      static std::vector<Edge2d> dummy;
      return dummy;
    }
    return pImpl->nearestNeighbor;
  }

  const std::vector<Triangle2d>& PointCloud::Triangulation() const
  {
    if (pImpl == nullptr)
    {
      static std::vector<Triangle2d> dummy;
      return dummy;
    }
    return pImpl->triangulation;
  }

  const std::vector<Triangle2d>& PointCloud::Delaunay() const
  {
    if (pImpl == nullptr)
    {
      static std::vector<Triangle2d> dummy;
      return dummy;
    }
    return pImpl->delaunay;
  }

  const std::vector<Edge2d>& PointCloud::Voronoi() const
  {
    if (pImpl == nullptr)
    {
      static std::vector<Edge2d> dummy;
      return dummy;
    }
    return pImpl->voronoi;
  }

  bool PointCloud::getBoundingBox(point3d& min, point3d& max) const
  {
    if (pImpl == nullptr) { return false; }
    if (pImpl->pointArray.empty()) { return false; }
    min = pImpl->pointArray[0];
    max = pImpl->pointArray[0];
    int startIndex = 1;
    for (int i = startIndex, NN = (int)(pImpl->pointArray.size()); i < NN; ++i)
    {
      const point3d& current = pImpl->pointArray[i];
      if (current.x < min.x) { min.x = current.x; }
      if (current.y < min.y) { min.y = current.y; }
      if (current.z < min.z) { min.z = current.z; }
      if (current.x > max.x) { max.x = current.x; }
      if (current.y > max.y) { max.y = current.y; }
      if (current.z > max.z) { max.z = current.z; }
    }
    return true;
  }

  void PointCloud::refresh(bool bRecompute)
  {
    if (pImpl == nullptr) { return; }
    // Generate random points for display.
    if (bRecompute)
    {
      pImpl->generateRandomPoints();
#ifdef __CUDA_RUNTIME_H__
      pImpl->computeHullDivideConquer();
#else
      computeConvexHull();
#endif
      pImpl->triangulation.resize(0);
      pImpl->delaunay.resize(0);
      pImpl->nearestNeighbor.resize(0);
      pImpl->voronoi.resize(0);
    }
    if (pImpl->bTrianglesModeOn)
    {
      if (pImpl->triangulation.empty())
      {
        pImpl->naiveTriangulate(pImpl->triangulation, pImpl->pointArray, pImpl->hull);
      }
    }
    if (pImpl->bDelaunayOn)
    {
      if (pImpl->delaunay.empty())
      {
        computeDelaunay();
      }
    }
    if (pImpl->bNearestNeighborOn)
    {
      if (pImpl->nearestNeighbor.empty())
      {
        pImpl->computeNearestNeighbor();
      }
    }
    if (pImpl->bVoronoiOn)
    {
      if (pImpl->voronoi.empty())
      {
        pImpl->computeVoronoi();
      }
    }
  }

  void PointCloud::setPoint(int index, const point2d& iPoint)
  {
    if (pImpl == nullptr) { return; }
    if (index < 0) { return; }
    if (index >= (int)(pImpl->pointArray.size())) { return; }
    pImpl->pointArray[index] = iPoint;
  }

  PointCloud& PointCloud::Get()
  {
    static PointCloud pc;
    return pc;
  }

  bool PointCloud::pointIsInConvexSorted(const std::vector<point2d>& iHull, const point2d& iPoint)
  {
    const int hullSize = (int)iHull.size();
    if (hullSize == 0) { return false; }
    if (hullSize == 1) { return (point2d::sqDistance(iHull[0], iPoint) <= threshold()); }
    if (hullSize == 2) { Edge2d edge(iHull[0], iHull[1]); return (edge.sqDistance(iPoint) <= threshold()); }
    int startIndex = 2;
    for (int ii = startIndex; ii < hullSize; ++ii)
    {
      Triangle2d face(iHull[0], iHull[ii - 1], iHull[ii]);
      if (face.pointIsInterior(iPoint) == 0) { continue; }
      return true;
    }
    return false;
  }

  void PointCloud::toggleConvexHull()
  {
    if (pImpl == nullptr) { return; }
    pImpl->bConvexHullOn = !(pImpl->bConvexHullOn);
  }

  void PointCloud::toggleDelaunay()
  {
    if (pImpl == nullptr) { return; }
    pImpl->bDelaunayOn = !(pImpl->bDelaunayOn);
  }

  void PointCloud::toggleNearestNeighbor()
  {
    if (pImpl == nullptr) { return; }
    pImpl->bNearestNeighborOn = !(pImpl->bNearestNeighborOn);
  }

  void PointCloud::togglePointsVisibility()
  {
    if (pImpl == nullptr) { return; }
    pImpl->bPointsVisible = !(pImpl->bPointsVisible);
  }

  void PointCloud::toggleTriangulation()
  {
    if (pImpl == nullptr) { return; }
    pImpl->bTrianglesModeOn = !(pImpl->bTrianglesModeOn);
  }

  void PointCloud::toggleVoronoi()
  {
    if (pImpl == nullptr) { return; }
    pImpl->bVoronoiOn = !(pImpl->bVoronoiOn);
  }

  bool PointCloud::convexHullIsOn() const
  {
    if (pImpl == nullptr) { return false; }
    return pImpl->bConvexHullOn;
  }

  bool PointCloud::delaunayIsOn() const
  {
    if (pImpl == nullptr) { return false; }
    return pImpl->bDelaunayOn;
  }

  bool PointCloud::nearestNeighborIsOn() const
  {
    if (pImpl == nullptr) { return false; }
    return pImpl->bNearestNeighborOn;
  }

  bool PointCloud::pointsAreOn() const
  {
    if (pImpl == nullptr) { return false; }
    return pImpl->bPointsVisible;
  }

  bool PointCloud::triangulationIsOn() const
  {
    if (pImpl == nullptr) { return false; }
    return pImpl->bTrianglesModeOn;
  }

  bool PointCloud::voronoiIsOn() const
  {
    if (pImpl == nullptr) { return false; }
    return pImpl->bVoronoiOn;
  }

  void PointCloud::computeDelaunay()
  {
    if (pImpl != nullptr) { pImpl->computeDelaunay(Impl::DelaunayMode::DivideAndConquer, pImpl->pointArray, pImpl->triangulation, pImpl->hull, pImpl->delaunay); }
  }

  void PointCloud::naiveTriangulate()
  {
    if (pImpl != nullptr) { pImpl->naiveTriangulate(pImpl->triangulation, pImpl->pointArray, pImpl->hull); }
  }

  void PointCloud::Impl::dividePointsInto3ConvexClouds(const std::vector<point2d>& iPointArray, std::vector<point2d>& oPointArray1, std::vector<point2d>& oPointArray2, std::vector<point2d>& oPointArray3)
  {
    std::vector<point2d> testHull;
    computeConvexHull(iPointArray, testHull);
    oPointArray1.resize(0);
    oPointArray2.resize(0);
    oPointArray3.resize(0);
    if (iPointArray.size() <= 3)
    {
      for (int ii = 0; ii < (int)(iPointArray.size()); ++ii)
      {
        if (ii == 0) { oPointArray1.push_back(iPointArray[0]); }
        else if (ii == 1) { oPointArray2.push_back(iPointArray[1]); }
        else if (ii == 2) { oPointArray3.push_back(iPointArray[2]); }
      }
      return;
    }
    std::sort(testHull.begin(), testHull.end(), point2d::comparator);
    if (testHull.size() == iPointArray.size()) // All points in hull.
    {
      int NN = (int)testHull.size() / 3;
      oPointArray2.push_back(testHull[0]);
      oPointArray3.push_back(testHull[0]);
      for (int ii = 0; ii < (int)(testHull.size()); ++ii)
      {
        if (ii <= NN) { oPointArray1.push_back(testHull[ii]); continue; }
        if (ii <= 2 * NN) { oPointArray2.push_back(testHull[ii]); continue; }
        oPointArray3.push_back(testHull[ii]);
      }
      return;
    }

    point2d centerOfMass(0, 0);
    {
#ifdef __CUDA_RUNTIME_H__
      int arrSize = (int)(iPointArray.size());
      std::vector<point2d> summands(iPointArray.size());
      point2d *d_PointArray, *d_PartialSums;
      int sizeToAlloc = sizeof(point2d) * arrSize;
      {
        // Allocate memory on the GPU.
        cudaMalloc((void**)&(d_PointArray), sizeToAlloc);
        cudaMalloc((void**)&(d_PartialSums), sizeToAlloc);
        cudaMemcpy(d_PointArray, iPointArray.data(), sizeToAlloc, cudaMemcpyHostToDevice);
      }

      int threadsPerBlock = 128;
      dim3 numBlocks = arrSize / threadsPerBlock;
      CenterOfMass <<<numBlocks, threadsPerBlock >>> (d_PointArray, arrSize, d_PartialSums);
      cudaMemcpy(summands.data(), d_PartialSums, sizeToAlloc, cudaMemcpyDeviceToHost);
      cudaFree(&d_PartialSums); // Free GPU memory.
      cudaFree(&d_PointArray); // Free GPU memory.
#else
      double inv = 1.0 / (double)(iPointArray.size());
      for (int ii = 0; ii < (int)(iPointArray.size()); ++ii)
      {
        centerOfMass.x = centerOfMass.x + iPointArray[ii].x * inv;
        centerOfMass.y = centerOfMass.y + iPointArray[ii].y * inv;
      }
#endif // def __CUDA_RUNTIME_H__
      double bestSqDist = -1;
      point2d bestPoint;
      for (int ii = 0; ii < (int)(iPointArray.size()); ++ii)
      {
        if (ii == 0)
        {
          bestSqDist = point2d::sqDistance(iPointArray[ii], centerOfMass);
          bestPoint = iPointArray[ii];
        }
        else if (point2d::sqDistance(iPointArray[ii], centerOfMass) < bestSqDist)
        {
          bestSqDist = point2d::sqDistance(iPointArray[ii], centerOfMass);
          bestPoint = iPointArray[ii];
        }
      }
      centerOfMass = bestPoint;
    }

    int bestScore = -1;
    // Trying each point is too slow. Use center of mass.
    // for (int ii = 0; ii < (int)(iPointArray.size()); ++ii)
    {
      //auto baseVertex = iPointArray[ii];
      auto& baseVertex = centerOfMass;
      for (int j1 = 0; j1 < (int)(testHull.size()); ++j1)
      for (int j2 = j1 + 1; j2 < (int)(testHull.size()); ++j2)
      for (int j3 = j2 + 1; j3 < (int)(testHull.size()); ++j3)
      {
        std::vector<point2d> testHull1, testHull2, testHull3;
        testHull1.push_back(baseVertex);
        testHull2.push_back(baseVertex);
        testHull3.push_back(baseVertex);
        for (int jj = 0; jj < (int)(testHull.size()); ++jj)
        {
          if (jj <= j1) { testHull1.push_back(testHull[jj]); continue; }
          if (jj <= j2) { testHull2.push_back(testHull[jj]); continue; }
          testHull3.push_back(testHull[jj]);
        }
        std::vector<point2d> tempArray1, tempArray2, tempArray3;
        for (int kk = 0; kk < (int)(iPointArray.size()); ++kk)
        {
          if (PointCloud::pointIsInConvexSorted(testHull1, iPointArray[kk]))
          {
            tempArray1.push_back(iPointArray[kk]); continue;
          }
          if (PointCloud::pointIsInConvexSorted(testHull2, iPointArray[kk]))
          {
            tempArray2.push_back(iPointArray[kk]); continue;
          }
          // if (PointCloud::pointIsInConvexSorted(testHull3, iPointArray[kk]))
          {
            tempArray3.push_back(iPointArray[kk]);
          }
        }
        int score1 = (int)tempArray1.size();
        int score2 = (int)tempArray2.size();
        int score3 = (int)tempArray3.size();
        int score = abs(score1 - score2) + abs(score2 - score3) + abs(score3 - score1);
        if ((bestScore < 0) || (score < bestScore))
        {
          bestScore = score;
          oPointArray1 = tempArray1;
          oPointArray2 = tempArray2;
          oPointArray3 = tempArray3;
        }
      }
    }
  }

#ifdef USE_MULTI_THREADING
  void PointCloud::Impl::computeDelaunayThreadFunc(int threadIndex)
  {
    //gMutex.lock();
    std::vector<Triangle2d> smallDelaunay;
    std::vector<Triangle2d> smallTriangulation;
    std::vector<point2d> smallHull;
    computeDelaunay(DelaunayMode::Naive, mSmallClouds[threadIndex], smallTriangulation, smallHull, smallDelaunay);
    for (auto& face : smallDelaunay)
    {
      mPartialTriangulations[threadIndex].push_back(face);
    }
    //gMutex.unlock();
  }
#endif // def USE_MULTI_THREADING

  void PointCloud::Impl::computeHullDivideConquer()
  {
    std::vector<point2d> tempArray = pointArray;
    hull = ConvexHullDivideAndConquer(tempArray);
  }

  void PointCloud::Impl::computeDelaunay(DelaunayMode delaunayMode, std::vector<point2d>& ioPointArray, std::vector<Triangle2d>& ioTriangulation, std::vector<point2d>& ioHull, std::vector<Triangle2d>& ioDelaunay)
  {
    if (delaunayMode == DelaunayMode::DivideAndConquer)
    {
      if ((int)(ioPointArray.size()) <= DelaunayMaxForNaive)
      {
        computeDelaunay(DelaunayMode::Naive, ioPointArray, ioTriangulation, ioHull, ioDelaunay);
        return;
      }

      // Lift to 3d space and reduce computation to convex hull.
      /*std::vector<point3d> pointArray3d(ioPointArray.size()); // TODO: Finish this.
      point3d bboxMin, bboxMax;
      for (int ii = 0; ii < ((int)ioPointArray.size()); ++ii)
      {
        const point2d& current = ioPointArray[ii];
        pointArray3d[ii] = point3d(current.x, current.y, current.sqNorm()); // Lift to 3d-space.
        if (ii == 0) { bboxMin = bboxMax = pointArray3d[ii]; }
        else
        {
          if (pointArray3d[ii].x < bboxMin.x) { bboxMin.x = pointArray3d[ii].x; }
          if (pointArray3d[ii].y < bboxMin.y) { bboxMin.y = pointArray3d[ii].y; }
          if (pointArray3d[ii].z < bboxMin.z) { bboxMin.z = pointArray3d[ii].z; }
          if (pointArray3d[ii].x > bboxMax.x) { bboxMax.x = pointArray3d[ii].x; }
          if (pointArray3d[ii].y > bboxMax.y) { bboxMax.y = pointArray3d[ii].y; }
          if (pointArray3d[ii].z > bboxMax.z) { bboxMax.z = pointArray3d[ii].z; }
        }
      }
      bboxMax.x = bboxMax.x + 1.0;
      bboxMax.y = bboxMax.y + 1.0;
      bboxMax.z = bboxMax.z + 1.0;
      bboxMin.x = bboxMin.x - 1.0;
      bboxMin.y = bboxMin.y - 1.0;
      bboxMin.z = bboxMin.z - 1.0;
      if (ioTriangulation.empty()) { naiveTriangulate(ioTriangulation, ioPointArray, ioHull); }
      auto hull3d = ConvexHull3d(pointArray3d);

      //ioDelaunay.push_back(ioTriangulation[0]);
      for (const auto& hullPoint : hull3d)
      {
      }

      return;*/

#ifdef USE_MULTI_THREADING
      auto& smallClouds = mSmallClouds;
      smallClouds.resize(0);
#else
      std::vector<std::vector<point2d> > smallClouds;
#endif // def USE_MULTI_THREADING
      {
        const int maxStackSize = 8;
        std::vector<std::vector<point2d> > remaining;
        {
          std::vector<point2d> oPointArray1, oPointArray2, oPointArray3;
          {
            dividePointsInto3ConvexClouds(ioPointArray, oPointArray1, oPointArray2, oPointArray3);
            if ((int)(oPointArray1.size()) <= DelaunayMaxForNaive)
            {
              smallClouds.push_back(oPointArray1);
            }
            else { remaining.push_back(oPointArray1); }
            if ((int)(oPointArray2.size()) <= DelaunayMaxForNaive)
            {
              smallClouds.push_back(oPointArray2);
            }
            else { remaining.push_back(oPointArray2); }
            if ((int)(oPointArray3.size()) <= DelaunayMaxForNaive)
            {
              smallClouds.push_back(oPointArray3);
            }
            else { remaining.push_back(oPointArray3); }
          }
          for (;remaining.size() > 0;)
          {
            if (remaining.size() >= maxStackSize)
            {
              for (const auto& trialCloud : remaining)
              {
                smallClouds.push_back(trialCloud);
              }
              break;
            }
            auto trialCloud = remaining[(int)remaining.size() - 1];
            remaining.resize((int)remaining.size() - 1);
            bool bFoundOne = false;
            {
              std::vector<point2d> thisRemainingCloud;
              dividePointsInto3ConvexClouds(trialCloud, oPointArray1, oPointArray2, oPointArray3);
              if ((int)(oPointArray1.size()) <= DelaunayMaxForNaive)
              {
                smallClouds.push_back(oPointArray1);
                bFoundOne = true;
              }
              else
              {
                for (const auto& thisPoint : oPointArray1)
                {
                  thisRemainingCloud.push_back(thisPoint);
                }
              }
              if ((int)(oPointArray2.size()) <= DelaunayMaxForNaive)
              {
                smallClouds.push_back(oPointArray2);
                bFoundOne = true;
              }
              else
              {
                for (const auto& thisPoint : oPointArray2)
                {
                  thisRemainingCloud.push_back(thisPoint);
                }
              }
              if ((int)(oPointArray3.size()) <= DelaunayMaxForNaive)
              {
                smallClouds.push_back(oPointArray3);
                bFoundOne = true;
              }
              else
              {
                for (const auto& thisPoint : oPointArray3)
                {
                  thisRemainingCloud.push_back(thisPoint);
                }
              }
              if (!bFoundOne)
              {
                remaining.push_back(oPointArray1);
                remaining.push_back(oPointArray2);
                remaining.push_back(oPointArray3);
              }
              else if (thisRemainingCloud.size() > 0)
              {
                remaining.push_back(thisRemainingCloud);
              }
            }
          }
        }
      }

      //std::cout << "\nNumber of small clouds: " << smallClouds.size() << std::endl;

      std::vector<Triangle2d> partialTriangulation;
#ifdef USE_MULTI_THREADING
      std::vector<std::thread> cloudThreads;
      mPartialTriangulations.resize(smallClouds.size());
      int threadIndex = 0;
#endif // USE_MULTI_THREADING
      for (auto& smallCloud : smallClouds)
      {
#ifdef USE_MULTI_THREADING
        std::thread cloudThread(&PointCloud::Impl::computeDelaunayThreadFunc, this, threadIndex);
        ++threadIndex;
        cloudThreads.push_back(std::move(cloudThread));
#else
        std::vector<Triangle2d> smallDelaunay;
        std::vector<Triangle2d> smallTriangulation;
        std::vector<point2d> smallHull;
        computeDelaunay(DelaunayMode::Naive, smallCloud, smallTriangulation, smallHull, smallDelaunay);
        for (auto& face : smallDelaunay)
        {
          partialTriangulation.push_back(face);
        }
#endif // USE_MULTI_THREADING
      }
#ifdef USE_MULTI_THREADING
      for (auto& cloudThread : cloudThreads) // Synchronize.
      {
        cloudThread.join();
      }
      for (auto& smallDelaunay : mPartialTriangulations)
      {
        for (const auto& face : smallDelaunay)
        {
          partialTriangulation.push_back(face);
        }
      }
#endif // USE_MULTI_THREADING
      computeDelaunay(DelaunayMode::Naive, ioPointArray, ioTriangulation, ioHull, ioDelaunay);
      return;
    }
    // if (delaunayMode == DelaunayMode::Naive):
    std::set<Triangle2d> faces;
    int numFaces = 0;
    {
      if (ioTriangulation.empty()) { naiveTriangulate(ioTriangulation, ioPointArray, ioHull); }
      if (ioPointArray.size() == 2)
      {
        if (ioTriangulation.size() >= 1)
        {
          ioDelaunay.push_back(ioTriangulation[0]);
          return;
        }
      }
      for (const auto& face : ioTriangulation)
      {
        faces.insert(face);
        ++numFaces;
      }
    }
    bool bDelaunayNotMet = false;
    int flips = 0;
    for (bool bStarting = true; bStarting || bDelaunayNotMet; bStarting = false)
    {
      bDelaunayNotMet = false;
      Triangle2d flip0, flip1;
      // Pre- C++ 11 style of iteration:
      std::set<Triangle2d>::iterator it = faces.begin();
      std::set<Triangle2d>::iterator jt = faces.begin();
      for (; it != faces.end(); ++it)
      {
        jt = faces.begin();
        for (; jt != faces.end(); ++jt)
        {
          if (it == jt) { continue; }

          Edge2d match;
          bool bIsAdjacent = it->adjacentToByEdge(*jt, match);
          if (!bIsAdjacent) { continue; }

          point2d iVertex = it->a;
          point2d jVertex = jt->a;
          if (match.sqDistance(iVertex) <= threshold()) { iVertex = it->b; }
          if (match.sqDistance(jVertex) <= threshold()) { jVertex = jt->b; }
          if (match.sqDistance(iVertex) <= threshold()) { iVertex = it->c; }
          if (match.sqDistance(jVertex) <= threshold()) { jVertex = jt->c; }
          if (match.sqDistance(iVertex) <= threshold()) { continue; }
          if (match.sqDistance(jVertex) <= threshold()) { continue; }

          Circle2d testCircle(it->a, it->b, it->c);
          if (testCircle.pointIsInterior(jVertex) == 1)
          {
            bDelaunayNotMet = true;
            flip0 = Triangle2d(iVertex, jVertex, match.a);
            flip1 = Triangle2d(iVertex, jVertex, match.b);
            ++flips;
          }
          if (!bDelaunayNotMet)
          {
            testCircle = Circle2d(jt->a, jt->b, jt->c);
            if (testCircle.pointIsInterior(iVertex) == 1)
            {
              bDelaunayNotMet = true;
              flip0 = Triangle2d(iVertex, jVertex, match.a);
              flip1 = Triangle2d(iVertex, jVertex, match.b);
              ++flips;
            }
          }
          if (bDelaunayNotMet) { break; }
        }
        if (bDelaunayNotMet) { break; }
      }
      if (bDelaunayNotMet)
      {
        faces.erase(it);
        faces.erase(jt);
        faces.insert(flip0);
        faces.insert(flip1);
      }
    }
    for (const auto& face : faces)
    {
      ioDelaunay.push_back(face);
    }
    //std::cout << "\nNumber of points: " << ioPointArray.size() << ".";
    //std::cout << "\nNumber of faces in initial triangulation: " << numFaces << ".";
    //std::cout << "\nNumber of Delaunay flips: " << flips << ".";
    //std::cout << "\nNumber of faces in Delaunay triangulation: " << ioDelaunay.size() << ".";
  }

  void PointCloud::Impl::naiveTriangulate(std::vector<Triangle2d>& ioTriangulation, std::vector<point2d>& ioPointArray, std::vector<point2d>& ioHull)
  {
    ioTriangulation.resize(0);
    if (ioHull.empty())
    {
      computeConvexHull(ioPointArray, ioHull);
    }
    int hullSize = (int) ioHull.size();
    if (hullSize <= 2)
    {
      if (ioPointArray.size() == 2)
      {
        if (point2d::sqDistance(ioPointArray[0], ioPointArray[1]) > threshold())
        {
          Triangle2d face(ioPointArray[0], ioPointArray[1], ioPointArray[0]);
          ioTriangulation.push_back(face);
        }
      }
      return;
    }
    int startIndex = 2;
    std::set<Triangle2d> faces;
    for (int i = startIndex; i < hullSize; ++i)
    {
      Triangle2d face(ioHull[0], ioHull[i - 1], ioHull[i]);
      faces.insert(face);
    }
    for (int i = 0, NN = (int)(ioPointArray.size()); i < NN; ++i)
    {
      // Pre- C++ 11 style of iteration:
      std::set<Triangle2d>::iterator it = faces.begin();
      point2d basePoint = ioPointArray[i];
      for (; it != faces.end(); ++it)
      {
        if ((it->pointIsInterior(basePoint)) == 1) { break; }
      }
      if (it == faces.end()) { continue; }
      Triangle2d u(basePoint, it->a, it->b);
      Triangle2d v(basePoint, it->b, it->c);
      Triangle2d w(basePoint, it->c, it->a);
      faces.erase(it);
      faces.insert(u);
      faces.insert(v);
      faces.insert(w);
    }
    for (const auto& face : faces)
    {
      ioTriangulation.push_back(face);
    }
  }

  void PointCloud::Impl::generateRandomPoints()
  {
    double randMax = (double)RAND_MAX;
    pointArray.resize(0);
    std::set<point2d> initialSet; // Using a set ensures unique points and allows automatic sorting.

    for (int i = 0; i < numRandomPoints(); ++i)
    {
      double xx = (rand() * 1.8 / randMax) - 0.9;
      double yy = (rand() * 1.8 / randMax) - 0.9;
      point2d P(xx, yy);
      initialSet.insert(P);
    }
  
    // Pre- C++ 11 style of iteration:
    for (std::set<point2d>::const_iterator it = initialSet.begin(); it != initialSet.end(); ++it)
    {
      pointArray.push_back(*it);
    }
  }

  void PointCloud::computeNearestNeighbor()
  {
    if (pImpl != nullptr) { pImpl->computeNearestNeighbor(); }
  }

  void PointCloud::Impl::computeNearestNeighbor()
  {
    nearestNeighbor.resize(0);
    if (delaunay.empty() && (pCloud != nullptr))
    {
      pCloud->computeDelaunay();
    }

    if (pointArray.size() == 2)
    {
      nearestNeighbor.push_back(Edge2d(pointArray[0], pointArray[1]));
      return;
    }

    std::set<Triangle2d> faces;
    for (const auto& face : delaunay)
    {
      faces.insert(face);
    }

    for (const auto& current : pointArray)
    {
      std::set<Edge2d> edgesForThisOne;
      for (const auto& face : faces)
      {
        if (point2d::sqDistance(face.a, current) <= threshold())
        {
          Edge2d edge(face.a, face.b);
          edgesForThisOne.insert(edge);
          edge = Edge2d(face.a, face.c);
          edgesForThisOne.insert(edge);
        }
        if (point2d::sqDistance(face.b, current) <= threshold())
        {
          Edge2d edge(face.a, face.b);
          edgesForThisOne.insert(edge);
          edge = Edge2d(face.b, face.c);
          edgesForThisOne.insert(edge);
        }
        if (point2d::sqDistance(face.c, current) <= threshold())
        {
          Edge2d edge(face.b, face.c);
          edgesForThisOne.insert(edge);
          edge = Edge2d(face.c, face.a);
          edgesForThisOne.insert(edge);
        }
      }
      if (edgesForThisOne.empty()) { continue; }
      Edge2d nearest = *(edgesForThisOne.begin());
      for (const auto& edge : edgesForThisOne)
      {
        if (edge.sqLength() < nearest.sqLength()) { nearest = edge; }
      }
      nearestNeighbor.push_back(nearest);
    }
  }

  void PointCloud::computeVoronoi()
  {
    if (pImpl != nullptr) { pImpl->computeVoronoi(); }
  }

  std::vector<Edge2d> PointCloud::Impl::constructVoronoiRays(const point2d& site, const std::vector<point2d>& localDelaunayVertices)
  {
    std::vector<Edge2d> voronoiRays;
    if (localDelaunayVertices.empty()) { return voronoiRays; }
    std::vector<Edge2d> delaunayEdges;
    for (int ii = 0; ii < (int)localDelaunayVertices.size(); ++ii)
    {
      int jj = ii + 1;
      if (jj >= (int)localDelaunayVertices.size()) { jj = 0; }
      delaunayEdges.push_back(Edge2d(pointArray[ii], pointArray[jj]));
    }
    for (const auto& delaunayEdge : delaunayEdges)
    {
      point2d proj = delaunayEdge.projection(site);
      voronoiRays.push_back(Edge2d(site, proj));
    }
    if (voronoiRays.size() < 3) { return voronoiRays; }
    for (int ii = 0; ii < 3; ++ii)
    {
      int jj = ii + 1;
      if (jj >= 3) { jj = 0; }
      int kk = 2;
      if (ii != 0) { kk = (ii == 1) ? 0 : 1; }
      auto testRay = voronoiRays[kk];
      auto minSqLen = voronoiRays[ii].sqLength();
      if (voronoiRays[jj].sqLength() < minSqLen) { minSqLen = voronoiRays[jj].sqLength(); }
      if (testRay.sqLength() > threshold())
      {
        double coeff = safeSqrt(minSqLen / testRay.sqLength()) / 2;
        testRay.b.x = coeff * (testRay.b.x - testRay.a.x) + testRay.a.x;
        testRay.b.y = coeff * (testRay.b.y - testRay.a.y) + testRay.a.y;
        Triangle2d testTri(site, voronoiRays[ii].b, voronoiRays[jj].b);
        if (testTri.pointIsInterior(testRay.b) == 0) { continue; }
        voronoiRays[kk].b.x = -(voronoiRays[kk].b.x - voronoiRays[kk].a.x) + voronoiRays[kk].a.x;
        voronoiRays[kk].b.y = -(voronoiRays[kk].b.y - voronoiRays[kk].a.y) + voronoiRays[kk].a.y;
        break;
      }
    }
    return voronoiRays;
  }

  void PointCloud::Impl::computeVoronoi2or3points()
  {
    if ((pointArray.size() < 2) || (pointArray.size() > 3))
    {
      return;
    }
    double raySqLength = -1.0;
    {
      Edge2d diag(gScreenMin, gScreenMax);
      raySqLength = diag.sqLength() * 4;
    }
    if (pointArray.size() == 2)
    {
      Edge2d seg(pointArray[0], pointArray[1]);
      point2d normal(seg.b.y - seg.a.y, seg.a.x - seg.b.x);
      point2d site((seg.a.x + seg.b.x) / 2, (seg.a.y + seg.b.y) / 2);
      voronoi.push_back(Edge2d(site, point2d(site.x + raySqLength * normal.x, site.y + raySqLength * normal.y)));
      voronoi.push_back(Edge2d(site, point2d(site.x - raySqLength * normal.x, site.y - raySqLength * normal.y)));
      return;
    }
      
    point2d site;
    bool bCollinear = false;
    try
    {
      Circle2d circ(pointArray[0], pointArray[1], pointArray[2]);
      site = circ.center;
    }
    catch (...) { bCollinear = true; }
    if (bCollinear) { return; } // TODO: There should be two lines in this case. Or treat it as a segment if two points coincide. Use recursion.
    if (delaunay.size() < 1) { return; } // In fact it should == 1.

    std::vector<Edge2d> voronoiRays = constructVoronoiRays(site, pointArray);
    for (const auto& voronoiRay : voronoiRays)
    {
      double coeff = raySqLength;
      double projSqLen = voronoiRay.sqLength();
      if (projSqLen > threshold()) { coeff = coeff / safeSqrt(projSqLen); }
      point2d rayPoint = point2d(coeff * (voronoiRay.b.x - voronoiRay.a.x) + voronoiRay.a.x, coeff * (voronoiRay.b.y - voronoiRay.a.y) + voronoiRay.a.y);
      Edge2d ray(site, rayPoint);
      voronoi.push_back(ray);
    }
  }

  void PointCloud::Impl::computeVoronoi()
  {
    voronoi.resize(0);
    if (delaunay.empty() && (pCloud != nullptr))
    {
      pCloud->computeDelaunay();
    }

    if (pointArray.size() <= 3)
    {
      computeVoronoi2or3points();
      return;
    }
      
    double raySqLength = -1.0;
    {
      Edge2d diag(gScreenMin, gScreenMax);
      raySqLength = diag.sqLength() * 4;
    }

    std::map<int, point2d> sites;
    std::map<int, Triangle2d> faces;
    {
      int i = 0;
      for (const auto& face : delaunay)
      {
        point2d site;
        bool bCollinear = false;
        try
        {
          Circle2d circ(face.a, face.b, face.c);
          site = circ.center;
        }
        catch (...)
        {
          bCollinear = true;
        }
        if (bCollinear) { continue; }
        faces[i] = face;
        sites[i] = site;
        ++i;
      }
    }

    for (const auto& siteIt : sites)
    {
      std::vector<point2d> sitesForThisOne;
      const auto& itsFace = faces[siteIt.first];
      const std::set<Edge2d> edges = itsFace.getEdges();
      std::set<Edge2d> nonMatching = edges;
      for (const auto& faceIt : faces)
      {
        if (faceIt.first == siteIt.first) { continue; }
        Edge2d match;
        bool bIsAdjacent = itsFace.adjacentToByEdge(faceIt.second, match);
        if (!bIsAdjacent) { continue; }
        sitesForThisOne.push_back(sites.at(faceIt.first));
        for (const auto& edge : edges)
        {
          if (match.sqDistance(edge.a) > threshold()) { continue; }
          if (match.sqDistance(edge.b) > threshold()) { continue; }
          nonMatching.erase(edge);
        }
        if (sitesForThisOne.size() >= 3) { break; }
      }

      double minSqLenFromSite = -1;
      std::vector<point2d> testEndpoints;
      for (const auto& endpoint : sitesForThisOne)
      {
        testEndpoints.push_back(endpoint);
        Edge2d edge(siteIt.second, endpoint);
        voronoi.push_back(edge);
        if ((minSqLenFromSite < 0) || (edge.sqLength() < minSqLenFromSite))
        {
          minSqLenFromSite = edge.sqLength();
        }
      }
      if (minSqLenFromSite < 0) { continue; }
      double testRaySqLen = minSqLenFromSite / 4;
      double testRayLen = safeSqrt(testRaySqLen); // Can we avoid sqrt?

      if (siteIt.second.x <= gScreenMin.x) { continue; }
      if (siteIt.second.y <= gScreenMin.y) { continue; }
      if (siteIt.second.x >= gScreenMax.x) { continue; }
      if (siteIt.second.y >= gScreenMax.y) { continue; }

      for (const auto& nonMatch : nonMatching)
      {
        point2d proj = nonMatch.projection(siteIt.second);
        double coeff = raySqLength;
          
        bool bShouldReverseRay = false;
        // Test to see: should we reverse ray?
        point2d testPoint(testRayLen * (proj.x - siteIt.second.x) + siteIt.second.x, testRayLen * (proj.y - siteIt.second.y) + siteIt.second.y);
        for (int siteInd = 0; siteInd < (int)testEndpoints.size(); ++siteInd)
        {
          int i0 = siteInd;
          int i1 = siteInd + 1;
          if (i1 >= (int)testEndpoints.size()) { i1 = 0; }
          Triangle2d testTri(siteIt.second, testEndpoints[i0], testEndpoints[i1]);
          if (testTri.pointIsInterior(testPoint)  == 1)
          {
            bShouldReverseRay = true; break;
          }
          testTri = Triangle2d(siteIt.second, testPoint, testEndpoints[i1]);
          if (testTri.pointIsInterior(testEndpoints[i0])  == 1)
          {
            bShouldReverseRay = true; break;
          }
          testTri = Triangle2d(siteIt.second, testPoint, testEndpoints[i0]);
          if (testTri.pointIsInterior(testEndpoints[i1])  == 1)
          {
            bShouldReverseRay = true; break;
          }
        }
        // Done testing.
        if (bShouldReverseRay) { coeff = -coeff; testRayLen = -testRayLen; }
        double projSqLen = Edge2d(proj, siteIt.second).sqLength();
        if (projSqLen > threshold()) { coeff = coeff / safeSqrt(projSqLen); }
        point2d rayPoint = point2d(coeff * (proj.x - siteIt.second.x) + siteIt.second.x, coeff * (proj.y - siteIt.second.y) + siteIt.second.y);
        Edge2d ray(siteIt.second, rayPoint);
        voronoi.push_back(ray);
        testEndpoints.push_back(point2d(testRayLen * (proj.x - siteIt.second.x) + siteIt.second.x, testRayLen * (proj.y - siteIt.second.y) + siteIt.second.y));
      }
    }
  }

  void PointCloud::computeConvexHull()
  {
    if (pImpl != nullptr) { pImpl->computeConvexHull(pImpl->pointArray, pImpl->hull); }
  }

  void PointCloud::Impl::computeConvexHull(const std::vector<point2d>& iPointArray, std::vector<point2d>& ioHull)
  {
    ioHull.resize(0);

    // 2d: Graham scan.
    ioHull = iPointArray;
    const int NN = (int)iPointArray.size();
    if (NN <= 3) { return; }

    {
      int bestIndex = 0;
      point2d tempOrigin = ioHull[0];
      for (int i = 0; i < NN; ++i)
      {
        if (ioHull[i].y >= tempOrigin.y) { continue; }
        bestIndex = i;
        tempOrigin = ioHull[bestIndex];
      }

      ioHull[bestIndex] = ioHull[1];
      ioHull[1] = tempOrigin;

      for (int i = 0; i < NN; ++i)
      {
        ioHull[i].x = ioHull[i].x - tempOrigin.x;
        ioHull[i].y = ioHull[i].y - tempOrigin.y;
      }
    
      // O(n log(n)):
      std::sort(ioHull.begin(), ioHull.end(), point2d::comparator);
    
      for (int i = 0; i < NN; ++i)
      {
        ioHull[i].x = ioHull[i].x + tempOrigin.x;
        ioHull[i].y = ioHull[i].y + tempOrigin.y;
      }
    }
    point2d endOfList = ioHull[0];
    
    int hullCount = 1; // Initialize stack.
    const int startIndex = 2;
    for (int i = startIndex; i <= NN; ++i)
    {
      point2d pointP = (hullCount == NN) ? endOfList : ioHull[hullCount];
      point2d pointQ = (i == NN) ? endOfList : ioHull[i];
      point2d pointO = (hullCount - 1 == NN) ? endOfList : ioHull[hullCount - 1];
      while (point2d::getOrientation(pointP, pointQ, pointO) <= 0)
      {
        if (hullCount > 1)
        {
          --hullCount;
          pointP = (hullCount == NN) ? endOfList : ioHull[hullCount];
          pointO = (hullCount - 1 == NN) ? endOfList : ioHull[hullCount - 1];
          continue;
        }
        if (i == NN) { break; } // Stack is empty.
        ++i; // Else keep searching.
        pointQ = (i == NN) ? endOfList : ioHull[i];
      }
      // Otherwise point is on the boundary of the convex hull.
      ++hullCount;
      pointP = (hullCount == NN) ? endOfList : ioHull[hullCount];
      pointO = (hullCount - 1 == NN) ? endOfList : ioHull[hullCount - 1];
      if (hullCount == NN) { endOfList = pointQ; }
      else { ioHull[hullCount] = pointQ; }
      if (i == NN) { endOfList = pointP; }
      else { ioHull[i] = pointP; }
    }

    ioHull.resize(hullCount);
  }

  template <class Container> double PointCloud::Impl::naiveMinSqDistance(Container& A, Container& B, point3d& A_min, point3d& B_min)
  {
    double min = 0;  bool started = false;
    for (typename Container::iterator it1 = A.begin(); it1 != A.end(); ++it1)
    {
      for (typename Container::iterator it2 = B.begin(); it2 != B.end(); ++it2)
      {
          // Note: set iteration takes place in sorted order.
          //std::cout << "[";  it2->print();  std::cout << "]\n";
          if (!started)
          {
              min = point3d::sqDistance(*it1, *it2);
              A_min = *it1;
              B_min = *it2;
              started = true;
              continue;
          }
          double candidate = point3d::sqDistance(*it1, *it2);
          if (candidate >= min) {continue;}
      
          min = candidate;
          A_min = *it1;
          B_min = *it2;
          if (min == 0) {break;}
      }
      if (min == 0) {break;}
    }
    return min;
  }

  template <class Container> double PointCloud::Impl::naiveMinSqDistance(Container& arr, point3d& min_1, point3d& min_2)
  {
    double min = 0;  bool started = false;
    if (arr.begin() != arr.end())
    {
      min_1 = *(arr.begin());
      min_2 = *(arr.begin());
    }

    // Iteration involving template parameters:
    for (typename Container::iterator it1 = arr.begin(); it1 != arr.end(); ++it1)
    {
      for (typename Container::iterator it2 = arr.begin(); it2 != arr.end(); ++it2)
      {
          if (it1 == it2) {continue;}
          // Note: set iteration takes place in sorted order.
          //std::cout << "[";  it2->print();  std::cout << "]\n";
          if (!started)
          {
              min = point3d::sqDistance(*it1, *it2);
              min_1 = *it1;
              min_2 = *it2;
              started = true;
              continue;
          }
          double candidate = point3d::sqDistance(*it1, *it2);
          if (candidate >= min) {continue;}
      
          min = candidate;
          min_1 = *it1;
          min_2 = *it2;
          if (min == 0) {break;}
      }
      if (min == 0) {break;}
    }
    return min;
  }

  double PointCloud::naiveMinSqDistance(std::set<point3d>& A, std::set<point3d>& B, point3d& A_min, point3d& B_min)
  {
    return Impl::naiveMinSqDistance(A, B, A_min, B_min);
  }

  double PointCloud::naiveMinSqDistance(std::set<point3d>& arr, point3d& min_1, point3d& min_2)
  {
    return Impl::naiveMinSqDistance(arr, min_1, min_2);
  }

  double PointCloud::naiveMinSqDistance(std::set<point2d>& A, std::set<point2d>& B, point2d& A_min, point2d& B_min)
  {
    return Impl::naiveMinSqDistance(A, B, A_min, B_min);
  }

  double PointCloud::naiveMinSqDistance(std::set<point2d>& cloud, point2d& min_1, point2d& min_2)
  {
    return Impl::naiveMinSqDistance(cloud, min_1, min_2);
  }

  double PointCloud::naiveMinSqDistance(point2d& min_1, point2d& min_2)
  {
    if (pImpl == nullptr) { return -1; }
    return Impl::naiveMinSqDistance(pImpl->pointArray, min_1, min_2);
  }

  template <class Container> double PointCloud::Impl::minSqDistanceHelper(Container& arr, point2d& min_1, point2d& min_2)
  {
    double min = 0;
    unsigned arrCount = (unsigned) arr.size();

    // These cases (where arrCount is 0 or 1) should never happen in the private helper method.
    //if (arrCount == 0) {return 0;}
    //if (arrCount == 1) {min_1 = *(arr.begin());  min_2 = *(arr.begin());  return 0;}
    if (arrCount == 2)
    {
      typename Container::iterator it = arr.begin();
      min_1 = *it;  ++it;  min_2 = *it;
      return point2d::sqDistance(min_1, min_2);
    }
    if (arrCount == 3)
    {
      typename Container::iterator it = arr.begin();
      point2d a = *it;  ++it;  point2d b = *it;
      double min_ = point2d::sqDistance(a, b);
      min_1 = a;  min_2 = b;
      ++it;
      double candidate = point2d::sqDistance(a, *it);
      if (candidate < min_)
      {
        min_ = candidate;  /*min_1 = a;*/  min_2 = *it;
      }
      candidate = point2d::sqDistance(*it, b);
      if (candidate < min_)
      {
        min_ = candidate;  min_1 = *it;  min_2 = b;
      }
      return min_;
    }
    
    unsigned halfArrCount = arrCount / 2;
    unsigned remainingArrCount = arrCount - halfArrCount;
    
    Container arr_1, arr_2;
    point2d left_1, left_2, right_1, right_2;
    double min_L, min_R;
    
    {
      typename Container::iterator it = arr.begin();
      for (int i = 0; i < (int)halfArrCount; i++)
      {
        arr_1.push_back(*it);  ++it;
      }
    
      for (int i = 0; i < (int)remainingArrCount; i++)
      {
        arr_2.push_back(*it);  ++it;
      }
    }
    
    min_L = minSqDistanceHelper(arr_1, left_1,  left_2);
    min_R = minSqDistanceHelper(arr_2, right_1, right_2);
    
    if (min_L < min_R)
    {
      min = min_L;
      min_1 = left_1;
      min_2 = left_2;
    }
    else
    {
      min = min_R;
      min_1 = right_1;
      min_2 = right_2;
    }
    return min;
  }

  template <class Container> double PointCloud::Impl::minSqDistance(Container& arr, point2d& min_1, point2d& min_2)
  {
    double min = 0;
    unsigned arrCount = (unsigned) arr.size();
    if (arrCount == 0) {return 0;}
    if (arrCount == 1) {min_1 = *(arr.begin());  min_2 = *(arr.begin());  return 0;}
    if (arrCount == 2)
    {
      typename Container::iterator it = arr.begin();
      min_1 = *it;  ++it;  min_2 = *it;
      return point2d::sqDistance(min_1, min_2);
    }
    if (arrCount == 3)
    {
      typename Container::iterator it = arr.begin();
      point2d a = *it;  ++it;  point2d b = *it;
      double min_ = point2d::sqDistance(a, b);
      min_1 = a;  min_2 = b;
      ++it;
      double candidate = point2d::sqDistance(a, *it);
      if (candidate < min_)
      {
        min_ = candidate;  /*min_1 = a;*/  min_2 = *it;
      }
      candidate = point2d::sqDistance(*it, b);
      if (candidate < min_)
      {
        min_ = candidate;  min_1 = *it;  min_2 = b;
      }
      return min_;
    }

    std::vector<point2d > arr_;
    for (typename Container::iterator it = arr.begin(); it != arr.end(); ++it)
    {
      arr_.push_back(*it);
    }
  
    std::sort(arr_.begin(), arr_.end());
    min = minSqDistanceHelper(arr_, min_1, min_2);
    return min;
  }

  double PointCloud::minSqDistance(std::set<point2d>& cloud, point2d& min_1, point2d& min_2)
  {
    return Impl::minSqDistance(cloud, min_1, min_2);
  }

  double PointCloud::minSqDistance(point2d& min_1, point2d& min_2)
  {
    if (pImpl == nullptr) { return -1; }
    return Impl::minSqDistance(pImpl->pointArray, min_1, min_2);
  }

  void PointCloud::unitTest()
  {
#ifdef USE_VIRTUAL_FUNC_POINT2D
    {
      point3d P(1.0, 2.0, 3.0);
      std::cout << "\n//////\nThe dimension is " << P.GetDimension() << ".";
      P.print("\n");
      std::cout << "\n";

      point2d Q(1.0, 2.0);
      std::cout << "\n//////\nThe dimension is " << Q.GetDimension() << ".";
      Q.print("\n");
      std::cout << "\n";
    }
#endif // def USE_VIRTUAL_FUNC_POINT2D
  
    {
      point3d P(1.0, 2.0, 3.0);
      point3d Q(1.0, -2.0, 3.0);
  
      printf("%f\n", point3d::sqDistance(P,Q));
    }
  
    {
      point2d P(1.5, 2.0);
      point2d Q(1.0, -2.0);
  
      printf("%f\n", point2d::sqDistance(P,Q));
    }
  
    {
      std::set<point3d> A;
      point3d a(1.5, 2.0, 0);
      point3d b(1.0, 3.0, 0);
      point3d c(-1.5, 7.0, 0);
    
      std::set<point3d> B;
      point3d d(4.5, 2.3, 0);
      point3d e(-1.55, 2.6, 0);
      point3d f(88.3, 0.001, 0);
    
      A.insert(a);
      A.insert(b);
      A.insert(c);
    
      B.insert(d);
      B.insert(e);
      B.insert(f);
    
      point3d p, q;
    
      double min = naiveMinSqDistance(A, B, p, q);
    
      std::cout << "\n//////\n" << min;
      p.print("\n");
      q.print("\n");
      std::cout << "\n";
    }
  
    {
      std::set<point2d> A;
      point2d a(1.5, 2.0);
      point2d b(1.0, 3.0);
      point2d c(-1.5, 7.0);
    
      std::set<point2d> B;
      point2d d(4.5, 2.3);
      point2d e(-1.35, 2.6);
      point2d f(88.3, 0.001);
    
      A.insert(a);
      A.insert(b);
      A.insert(c);
    
      B.insert(d);
      B.insert(e);
      B.insert(f);
    
      point2d p, q;
    
      double min = naiveMinSqDistance(A, B, p, q);
    
      std::cout << "\n//////\n" << min << "\n";
      p.print("\n");
      q.print("\n");
      std::cout << "\n";
    }
  
    {
      std::set<point2d > A;
      point2d a(1.5, 2.0);
      point2d b(1.0, 3.0);
      point2d c(-1.5, 7.0);
      point2d d(4.5, 2.3);
      point2d e(-1.35, 2.6);
      point2d f(88.3, 0.001);
    
      A.insert(a);
      A.insert(b);
      A.insert(c);
      A.insert(d);
      A.insert(e);
      A.insert(f);
    
      point2d p, q;

      double min = naiveMinSqDistance(A, p, q);
    
      std::cout << "\n//////\n";
      std::cout << min << "\n";
      p.print();  std::cout << "\n";
      q.print();  std::cout << "\n";
    
      min = minSqDistance(A, p, q);
    
      std::cout << "\n/!!!!/\n";
      std::cout << min << "\n";
      p.print();  std::cout << "\n";
      q.print();  std::cout << "\n";
    }
  } // end of unit_test function.

} // end of namespace ComputationalGeometry
