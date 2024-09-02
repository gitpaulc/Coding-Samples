/*  Copyright Paul Cernea, September 2024.
All Rights Reserved.*/

#include "create_video.h"

#include "point_cloud.h"

#ifdef USE_OPEN_CV

#include <opencv2/opencv.hpp>

namespace ComputationalGeometry
{
  bool createVideo(std::string& errMsg, VideoMode vm)
  {
    typedef cv::Point3_<uint8_t> Pixel;
    cv::VideoCapture capture(0);
    if (!capture.isOpened()) { errMsg = "Failed to open video stream.";  return false; }

    const auto frameWidth = (int)capture.get(cv::CAP_PROP_FRAME_WIDTH);
    const auto frameHeight = (int)capture.get(cv::CAP_PROP_FRAME_HEIGHT);

    cv::VideoWriter videoOut("../video_output/compGeo.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 10, cv::Size(frameWidth, frameHeight));

    uint8_t fewerColorsInterval = 30;

    const char escapeKey = (char)27;
    while (true)
    {
      cv::Mat currentFrame;
      capture >> currentFrame;
      if (currentFrame.empty()) { break; }
      if ((vm == VideoMode::Grayscale) || (vm == VideoMode::FewerColors))
      {
        //auto prev = currentFrame;
        //cv::cvtColor(prev, currentFrame, cv::COLOR_RGB2GRAY);
        for (int r = 0; r < currentFrame.rows; ++r) {
            Pixel* ptr = currentFrame.ptr<Pixel>(r, 0);
            const Pixel* ptr_end = ptr + currentFrame.cols;
            for (; ptr != ptr_end; ++ptr) {
                if (vm == VideoMode::FewerColors)
                {
                  ptr->x = (ptr->x / fewerColorsInterval) * fewerColorsInterval;
                  ptr->y = (ptr->y / fewerColorsInterval) * fewerColorsInterval;
                  ptr->z = (ptr->z / fewerColorsInterval) * fewerColorsInterval;
                  continue;
                }
                int avg = (ptr->x + ptr->y + ptr->z) / 3;
                ptr->x = (uint8_t)avg;
                ptr->y = (uint8_t)avg;
                ptr->z = (uint8_t)avg;
            }
        }
      }
      videoOut.write(currentFrame);
      cv::imshow("Video", currentFrame); // Display current frame.
      char keyboardKey = (char)cv::waitKey(1);
      if (keyboardKey == escapeKey) { break; }
    }

    return true;
  }
}

#endif // def USE_OPEN_CV