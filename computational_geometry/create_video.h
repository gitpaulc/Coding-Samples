/*  Copyright Paul Cernea, September 2024.
All Rights Reserved.*/

#ifndef CREATE_VIDEO_H
#define CREATE_VIDEO_H

#include "includes.h"

#ifdef USE_OPEN_CV
namespace ComputationalGeometry
{
  enum class VideoMode
  {
    Usual = 0,
    Grayscale = 1,
    FewerColors = 2
  };
  /** \return `true` upon success. */
  bool createVideo(std::string& errMsg, VideoMode vm = VideoMode::Usual);
}
#endif // def USE_OPEN_CV

#endif //def CREATE_VIDEO_H
