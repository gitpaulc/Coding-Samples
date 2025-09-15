/*  Copyright Paul Cernea, August 2025.
All Rights Reserved.*/

#include "image.h"
#include <fstream>

namespace MeshRenderer
{
  Image::Image(const std::string& filename)
  {
    if (false) // Test if file exists.
    {
      std::ifstream istr(filename);
      if (!(istr.is_open())) { return; }
    }
#ifdef _WIN64
    // Win64:
    hBitmap = (HBITMAP)LoadImage(
      NULL, filename.c_str(), IMAGE_BITMAP, 0, 0,
      LR_LOADFROMFILE | LR_CREATEDIBSECTION);
#else
#ifdef _WIN32
    // Win32:
    hBitmap = (HBITMAP)LoadImage(
      NULL, filename.c_str(), IMAGE_BITMAP, 0, 0,
      LR_LOADFROMFILE | LR_CREATEDIBSECTION);
#else
    // Otherwise...
#endif
#endif
  }

  Image::~Image()
  {
#ifdef _WIN64
    // Win64:
    if (hBitmap != false)
    {
      DeleteObject(hBitmap);
    }
#else
#ifdef _WIN32
    // Win32:
    if (hBitmap != false)
    {
      DeleteObject(hBitmap);
    }
#else
    // Otherwise...
#endif
#endif
  }

#ifdef _WIN64
  BITMAP Image::GetBitmap()
  {
    BITMAP bmp = {};
    if (hBitmap != false)
    {
      GetObject(hBitmap, sizeof(BITMAP), &bmp);
    }
    return bmp;
  }
#else
#ifdef _WIN32
  // Win32:
  BITMAP Image::GetBitmap()
  {
    BITMAP bmp = {};
    if (hBitmap != false)
    {
      GetObject(hBitmap, sizeof(BITMAP), &bmp);
    }
    return bmp;
  }
#else
  // Otherwise...
#endif
#endif

}
