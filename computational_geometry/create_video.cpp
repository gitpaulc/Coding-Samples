/*  Copyright Paul Cernea, September 2024.
All Rights Reserved.*/

#include "create_video.h"

#include "point_cloud.h"

#ifdef USE_OPEN_CV

#ifndef IMG_OUTPUT_FOLDER
#define IMG_OUTPUT_FOLDER ".."
#endif // not IMG VIDEO_OUTPUT_FOLDER

#ifndef VIDEO_OUTPUT_FOLDER
#define VIDEO_OUTPUT_FOLDER ".."
#endif // not def VIDEO_OUTPUT_FOLDER

#include <opencv2/opencv.hpp>
#include <iostream>

namespace ComputationalGeometry
{
  // Declarations:

  bool createVideo(std::string& errMsg, VideoMode vm);
  /** \brief Canny edge detection. */
  cv::Mat detectEdges(const cv::Mat& imgIn);
  cv::Mat kMeansClustering(const cv::Mat& imgIn, int k);
  int sqDist(const cv::Point3_<uint8_t>& pixA, const cv::Point3_<uint8_t>& pixB);

  // Implementations:

  /** \brief Canny edge detection. */
  cv::Mat detectEdges(const cv::Mat& imgIn)
  {
    int low = 30; // 0 - 100
    if (low < 0) { low = 0; }
    else if (low > 100) { low = 100; }
    int high = 3 * low;
    if (high < low) { high = low; }
    else if (high > 300) { high = 300; }
    const int kernelSize = 3;
    const int blurSize = kernelSize;

    cv::Mat imgOut, gray, edges;
    imgOut.create(imgIn.size(), imgIn.type());
    cv::cvtColor(imgIn, gray, cv::COLOR_BGR2GRAY);
    cv::blur(gray, edges, cv::Size(blurSize, blurSize));
    cv::Canny(edges, edges, low, high, kernelSize);
    imgOut = cv::Scalar::all(255);
    imgIn.copyTo(imgOut, edges);
    return imgOut;
  }

  static std::vector<cv::Point3_<uint8_t> > standardColors(bool few = false)
  {
    typedef cv::Point3_<uint8_t> Pixel;
    std::vector<Pixel> colors;
    if (few)
    {
      colors.push_back(Pixel(235, 51, 36)); // Red
      colors.push_back(Pixel(119, 67, 66)); // Brown
      colors.push_back(Pixel(115, 251, 253)); // Sky Blue
      colors.push_back(Pixel(50, 130, 246)); // Blue
      colors.push_back(Pixel(255, 253, 85)); // Yellow
      colors.push_back(Pixel(240, 155, 89)); // Light Peach
      colors.push_back(Pixel(115, 43, 245)); // Purple
      colors.push_back(Pixel(255, 127, 39)); // Orange
      colors.push_back(Pixel(117, 249, 77)); // Light Green
      colors.push_back(Pixel(238, 138, 248)); // Lavender
      colors.push_back(Pixel(234, 63, 247)); // Hot Pink
      colors.push_back(Pixel(55, 125, 34)); // Green
      colors.push_back(Pixel(0, 0, 0)); // Black
      colors.push_back(Pixel(128, 128, 128)); // Gray
      colors.push_back(Pixel(192, 192, 192)); // Light Gray
      colors.push_back(Pixel(255, 255, 255)); // White
      return colors;
    }
// #define EXTRA_COLORS
    colors.push_back(Pixel(240, 135, 132)); // Rose
    colors.push_back(Pixel(235, 51, 36)); // Red
    colors.push_back(Pixel(119, 67, 66)); // Brown
    colors.push_back(Pixel(142, 64, 58)); // Brownish
    colors.push_back(Pixel(58, 6, 3)); // Dark Red
    colors.push_back(Pixel(159, 252, 253)); // Sky Blue
    colors.push_back(Pixel(115, 251, 253)); // Sky Blue
    colors.push_back(Pixel(50, 130, 246)); // Blue
    colors.push_back(Pixel(0, 35, 245)); // True Blue
    colors.push_back(Pixel(0, 18, 154)); // Dark Blue
    colors.push_back(Pixel(22, 65, 124)); // Dark Navy
    colors.push_back(Pixel(0, 12, 123)); // Darkest Blue

    colors.push_back(Pixel(255, 254, 145)); // Light Yellow
    colors.push_back(Pixel(255, 253, 85)); // Yellow
    colors.push_back(Pixel(240, 155, 89)); // Light Peach
    colors.push_back(Pixel(240, 134, 80)); // Peach
    colors.push_back(Pixel(120, 67, 21)); // True Brown
    colors.push_back(Pixel(129, 127, 38)); // Gold
    colors.push_back(Pixel(126, 132, 247)); // Mauve
    colors.push_back(Pixel(115, 43, 245)); // Purple
    colors.push_back(Pixel(53, 128, 187)); // Teal
#ifdef EXTRA_COLORS
    colors.push_back(Pixel(0, 2, 61)); // Almost Black
#endif // def EXTRA_COLORS
    colors.push_back(Pixel(88, 19, 94)); // Dark Purple
    colors.push_back(Pixel(58, 8, 62)); // Darker Purple

#ifdef EXTRA_COLORS
    colors.push_back(Pixel(161, 251, 142)); // Lightest Green
    colors.push_back(Pixel(161, 250, 79)); // Lighter Green
#endif // def EXTRA_COLORS
    colors.push_back(Pixel(117, 249, 77)); // Light Green
#ifdef EXTRA_COLORS
    colors.push_back(Pixel(117, 250, 97)); // Light Green
    colors.push_back(Pixel(117, 250, 141)); // Light Green
#endif // def EXTRA_COLORS
    colors.push_back(Pixel(129, 128, 73)); // Olive-Gold
#ifdef EXTRA_COLORS
    colors.push_back(Pixel(239, 136, 190)); // Cotton Candy Pink
#endif // def EXTRA_COLORS
    colors.push_back(Pixel(238, 138, 248)); // Lavender
    colors.push_back(Pixel(234, 63, 247)); // Hot Pink
    colors.push_back(Pixel(234, 54, 128)); // Reddish Pink
    colors.push_back(Pixel(127, 130, 187)); // Blue-gray
    colors.push_back(Pixel(117, 22, 63)); // Plum

    colors.push_back(Pixel(55, 125, 34)); // Green
    colors.push_back(Pixel(55, 126, 71)); // Forest
    colors.push_back(Pixel(54, 126, 127)); // Dark Teal
    colors.push_back(Pixel(80, 127, 128)); // Teal
    colors.push_back(Pixel(24, 62, 12)); // Dark Green
#ifdef EXTRA_COLORS
    colors.push_back(Pixel(23, 63, 63)); // Dark Teal
#endif // def EXTRA_COLORS
    colors.push_back(Pixel(116, 27, 124)); // Dark Purple
    colors.push_back(Pixel(57, 16, 123)); // Crayon Purple
    colors.push_back(Pixel(0, 0, 0)); // Black
    colors.push_back(Pixel(128, 128, 128)); // Gray
    colors.push_back(Pixel(192, 192, 192)); // Light Gray
    colors.push_back(Pixel(255, 255, 255)); // White

    colors.push_back(Pixel(255, 127, 39)); // Orange
    colors.push_back(Pixel(185, 122, 87)); // Brown
    colors.push_back(Pixel(0, 162, 232)); // Turquoise
    return colors;
  }

  cv::Mat kMeansClustering(const cv::Mat& imgIn, int k)
  {
    if (k <= 0) { k = 1; }
    cv::Mat imgOut = imgIn.clone();
    if (k >= imgOut.rows * imgOut.cols) { return imgOut; }
    typedef cv::Point3_<uint8_t> Pixel;
    auto centroids = std::vector<Pixel>(k);
    std::vector<cv::Point3_<uint> > clusterSum(k);
    std::vector<int> clusterCount(k);
    {
      int ind = 0; int counter = -1;
      int spacing = imgOut.rows * imgOut.cols / k;
      if (spacing <= 0) { spacing = 1; }
      for (int row = 0; row < imgOut.rows; ++row)
      {
        Pixel* pix = imgOut.ptr<Pixel>(row, 0);
        const Pixel* row_end = pix + imgOut.cols;
        for (; pix != row_end; ++pix)
        {
          if (ind >= k) { break; }
          ++counter;
          if ((spacing > 1) && ((counter % spacing) != 0)) { continue; }
          centroids[ind] = *pix;
          ++ind;
        }
        if (ind >= k) { break; }
      }
    }
    bool foundCentroids = false;
    int prevDiff = -1;
    while (!foundCentroids)
    {
      foundCentroids = true;
      for (int ind = 0; ind < k; ++ind)
      {
        clusterSum[ind].x = 0;
        clusterSum[ind].y = 0;
        clusterSum[ind].z = 0;
        clusterCount[ind] = 0;
      }
      for (int row = 0; row < imgOut.rows; ++row)
      {
        Pixel* pix = imgOut.ptr<Pixel>(row, 0);
        const Pixel* row_end = pix + imgOut.cols;
        for (; pix != row_end; ++pix)
        {
          int bestSqDist = sqDist(centroids[0], *pix);
          int bestInd = 0;
          for (int ind = 0; ind < k; ++ind)
          {
            auto current = sqDist(centroids[ind], *pix);
            if (current >= bestSqDist) { continue; }
            bestSqDist = current;
            bestInd = ind;
          }
          clusterSum[bestInd].x += pix->x;
          clusterSum[bestInd].y += pix->y;
          clusterSum[bestInd].z += pix->z;
          clusterCount[bestInd]++;
        }
      }
      std::vector<Pixel> newCentroids(k);
      for (int ind = 0; ind < k; ++ind)
      {
        int siz = clusterCount[ind];
        if (siz == 0) { newCentroids[ind] = centroids[ind]; continue; }
        int xx = clusterSum[ind].x / siz;
        int yy = clusterSum[ind].y / siz;
        int zz = clusterSum[ind].z / siz;
        newCentroids[ind].x = (uint8_t)xx;
        newCentroids[ind].y = (uint8_t)yy;
        newCentroids[ind].z = (uint8_t)zz;
      }
      int diff = 0;
      for (int ind = 0; ind < k; ++ind)
      {
        int du = sqDist(centroids[ind], newCentroids[ind]);
        if (du == 0) { continue; }
        diff += du;
        foundCentroids = false; break;
      }
      if (prevDiff == diff) { break; }
      prevDiff = diff;
      if (foundCentroids) { break; }
      for (int ind = 0; ind < k; ++ind) { centroids[ind] = newCentroids[ind]; }
    }
    // Update image:
    for (int row = 0; row < imgOut.rows; ++row)
    {
      Pixel* pix = imgOut.ptr<Pixel>(row, 0);
      const Pixel* row_end = pix + imgOut.cols;
      for (; pix != row_end; ++pix)
      {
        int bestSqDist = sqDist(centroids[0], *pix);
        int bestInd = 0;
        for (int ind = 0; ind < k; ++ind)
        {
          auto current = sqDist(centroids[ind], *pix);
          if (current >= bestSqDist) { continue; }
          bestSqDist = current;
          bestInd = ind;
        }
        pix->x = centroids[bestInd].x;
        pix->y = centroids[bestInd].y;
        pix->z = centroids[bestInd].z;
      }
    }
    return imgOut;
  }

  int sqDist(const cv::Point3_<uint8_t>& pixA, const cv::Point3_<uint8_t>& pixB)
  {
    int sqDistCalc = 0;
    int dt = (pixA.x - pixB.x); dt *= dt; sqDistCalc += dt;
    dt = (pixA.y - pixB.y); dt *= dt; sqDistCalc += dt;
    dt = (pixA.z - pixB.z); dt *= dt; sqDistCalc += dt;
    return sqDistCalc;
  }

  cv::Mat simpleImage(const cv::Mat& imgIn, bool verySimple = false)
  {
    cv::Mat imgOut = imgIn.clone();
    typedef cv::Point3_<uint8_t> Pixel;
    const auto simpleColors = standardColors(verySimple);
    const auto numSimpleColors = (int)simpleColors.size();
    for (int row = 0; row < imgOut.rows; ++row)
    {
      Pixel* pix = imgOut.ptr<Pixel>(row, 0);
      const Pixel* row_end = pix + imgOut.cols;
      for (; pix != row_end; ++pix)
      {
        int bestSqDist = sqDist(simpleColors[0], *pix);
        int bestInd = 0;
        for (int ind = 0; ind < numSimpleColors; ++ind)
        {
          auto current = sqDist(simpleColors[ind], *pix);
          if (current >= bestSqDist) { continue; }
          bestSqDist = current;
          bestInd = ind;
        }
        pix->x = simpleColors[bestInd].x;
        pix->y = simpleColors[bestInd].y;
        pix->z = simpleColors[bestInd].z;
      }
    }
    return imgOut;
  }

  bool createVideo(std::string& errMsg, VideoMode vm)
  {
    typedef cv::Point3_<uint8_t> Pixel;

    int frameWidth = 0;
    int frameHeight = 0;

    int numKMeansClusters = 6; // 30 is slower.

    std::string videoOutFolder = VIDEO_OUTPUT_FOLDER;

    uint8_t fewerColorsInterval = 30;

    const char escapeKey = (char)27;
    std::vector<cv::Mat> arrayOfFrames;
    {
      cv::VideoCapture capture(0);
      if (!capture.isOpened()) { errMsg = "Failed to open video stream.";  return false; }
      frameWidth = (int)capture.get(cv::CAP_PROP_FRAME_WIDTH);
      frameHeight = (int)capture.get(cv::CAP_PROP_FRAME_HEIGHT);

      cv::Mat currentFrame;
      cv::VideoWriter videoOut(videoOutFolder + "/compGeo.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 10, cv::Size(frameWidth, frameHeight));
      while (true)
      {
        capture >> currentFrame;
        if (currentFrame.empty()) { break; }
        if (vm == VideoMode::Usual)
        {
          arrayOfFrames.push_back(currentFrame.clone());
        }
        else if ((vm == VideoMode::Grayscale) || (vm == VideoMode::FewerColors))
        {
          for (int row = 0; row < currentFrame.rows; ++row)
          {
            Pixel* pix = currentFrame.ptr<Pixel>(row, 0);
            const Pixel* row_end = pix + currentFrame.cols;
            for (; pix != row_end; ++pix)
            {
              if (vm == VideoMode::FewerColors)
              {
                pix->x = (pix->x / fewerColorsInterval) * fewerColorsInterval;
                pix->y = (pix->y / fewerColorsInterval) * fewerColorsInterval;
                pix->z = (pix->z / fewerColorsInterval) * fewerColorsInterval;
                continue;
              }
              int avg = (pix->x + pix->y + pix->z) / 3;
              pix->x = (uint8_t)avg;
              pix->y = (uint8_t)avg;
              pix->z = (uint8_t)avg;
            }
          }
        }
        else if (vm == VideoMode::EdgeDetection)
        {
          currentFrame = detectEdges(currentFrame.clone());
        }
        videoOut.write(currentFrame);
        cv::imshow("Video - Press ESC to exit.", currentFrame); // Display current frame.
        char keyboardKey = (char)cv::waitKey(1);
        if (keyboardKey == escapeKey) { break; }
      }
      std::string imgOutFolder = IMG_OUTPUT_FOLDER;
      cv::imwrite(imgOutFolder + "/compGeoScreenshot.png", currentFrame);
      // Edge detection.
      cv::imwrite(imgOutFolder + "/compGeoEdges.png", detectEdges(currentFrame));
      // Very simple colors.
      cv::imwrite(imgOutFolder + "/compGeoVerySimple.png", simpleImage(currentFrame, true));
      // Simple colors.
      cv::imwrite(imgOutFolder + "/compGeoSimple.png", simpleImage(currentFrame));
      // K-means clustering.
      cv::imwrite(imgOutFolder + "/compGeoKmeans.png", kMeansClustering(currentFrame, numKMeansClusters));
    }
    if (vm == VideoMode::Usual)
    {
      // Grayscale.
      {
        cv::VideoWriter videoOut(videoOutFolder + "/compGeoGrayscale.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 10, cv::Size(frameWidth, frameHeight));
        for (const auto& currentFrame : arrayOfFrames)
        {
          cv::Mat src = currentFrame.clone();
          cv::Mat dst = currentFrame.clone();
          cv::cvtColor(src, dst, cv::COLOR_RGB2GRAY);
          cv::cvtColor(dst, src, cv::COLOR_GRAY2RGB);
          videoOut.write(src);
        }
      }
      // Edges
      {
        cv::VideoWriter videoOut(videoOutFolder + "/compGeoEdges.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 10, cv::Size(frameWidth, frameHeight));
        for (const auto& currentFrame : arrayOfFrames)
        {
          videoOut.write(detectEdges(currentFrame.clone()));
        }
      }
      // Very simple colors.
      {
        cv::VideoWriter videoOut(videoOutFolder + "/compGeoVerySimple.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 10, cv::Size(frameWidth, frameHeight));
        for (const auto& currentFrame : arrayOfFrames)
        {
          videoOut.write(simpleImage(currentFrame.clone(), true));
        }
      }
      // Simple colors.
      {
        cv::VideoWriter videoOut(videoOutFolder + "/compGeoSimple.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 10, cv::Size(frameWidth, frameHeight));
        for (const auto& currentFrame : arrayOfFrames)
        {
          videoOut.write(simpleImage(currentFrame.clone()));
        }
      }
      // K-Means Clustering
      {
        cv::VideoWriter videoOut(videoOutFolder + "/compGeoKmeans.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 10, cv::Size(frameWidth, frameHeight));
        cv::VideoWriter videoOut2(videoOutFolder + "/compGeoKmeansEdges.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 10, cv::Size(frameWidth, frameHeight));
        for (const auto& currentFrame : arrayOfFrames)
        {
          auto kMeans = kMeansClustering(currentFrame.clone(), numKMeansClusters);
          videoOut.write(kMeans);
          videoOut2.write(detectEdges(kMeans.clone()));
        }
      }
    }

    std::cout << "\nNum frames total = " << arrayOfFrames.size();
    return true;
  }
}

#endif // def USE_OPEN_CV
