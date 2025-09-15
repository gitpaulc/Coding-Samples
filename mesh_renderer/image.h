/*  Copyright Paul Cernea, August 2025.
All Rights Reserved.*/

#ifndef MESH_IMAGE_H
#define MESH_IMAGE_H

#include "includes.h"

namespace MeshRenderer
{
  class Image
  {
#ifdef _WIN64
    HBITMAP hBitmap = NULL;
#else
#ifdef _WIN32
    // Win32:
    HBITMAP hBitmap = NULL;
#else
    // Otherwise...
    long imgH = 0;
    long imgW = 0;
#endif
#endif
  public:
    Image(const std::string& filename);
    virtual ~Image();
#ifdef _WIN64
    BITMAP GetBitmap();
#else
#ifdef _WIN32
    // Win32:
    BITMAP GetBitmap();
#else
    // Otherwise...
#endif
#endif
  };
}

#endif //def MESH_IMAGE_H
