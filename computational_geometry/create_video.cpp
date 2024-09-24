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
  bool createVideo(std::string& errMsg, VideoMode vm)
  {
    typedef cv::Point3_<uint8_t> Pixel;

    int frameWidth = 0;
    int frameHeight = 0;

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
        if ((vm == VideoMode::Grayscale) || (vm == VideoMode::FewerColors))
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
        videoOut.write(currentFrame);
        cv::imshow("Video - Press ESC to exit.", currentFrame); // Display current frame.
        char keyboardKey = (char)cv::waitKey(1);
        if (keyboardKey == escapeKey) { break; }
      }
      std::string imgOutFolder = IMG_OUTPUT_FOLDER;
      cv::imwrite(imgOutFolder + "/compGeoScreenshot.png", currentFrame);
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
    }

    std::cout << "\nNum frames total = " << arrayOfFrames.size();
    return true;
  }
}

#endif // def USE_OPEN_CV
