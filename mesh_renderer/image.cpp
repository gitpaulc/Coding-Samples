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
    HBITMAP hBitmap = (HBITMAP)LoadImage(
      NULL, filename.c_str(), IMAGE_BITMAP, 0, 0,
      LR_LOADFROMFILE | LR_CREATEDIBSECTION);

    if (hBitmap != false)
    {
      GetObject(hBitmap, sizeof(BITMAP), &bmp);
      DeleteObject(hBitmap);
    }
#else
#ifdef _WIN32
    // Win32:
    HBITMAP hBitmap = (HBITMAP)LoadImage(
      NULL, filename.c_str(), IMAGE_BITMAP, 0, 0,
      LR_LOADFROMFILE | LR_CREATEDIBSECTION);

    if (hBitmap != false)
    {
      GetObject(hBitmap, sizeof(BITMAP), &bmp);
      DeleteObject(hBitmap);
    }
#else
    // Otherwise...
#endif
#endif
  }
}
