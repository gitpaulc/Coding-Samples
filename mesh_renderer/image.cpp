/*  Copyright Paul Cernea, August 2025.
All Rights Reserved.*/

#include "image.h"
#include <fstream>

namespace MeshRenderer
{
  void* loadImage(const std::string& filename, long& imgW, long& imgH)
  {
    if (false) // Test if file exists.
    {
      std::ifstream istr(filename);
      if (!(istr.is_open())) { return nullptr; }
    }
#ifdef _WIN64
    // Win64:
    HBITMAP hBitmap = (HBITMAP)LoadImage(
      NULL, filename.c_str(), IMAGE_BITMAP, 0, 0,
      LR_LOADFROMFILE | LR_CREATEDIBSECTION);

    if (hBitmap == false)
    {
      return nullptr;
    }

    BITMAP bmp = {};
    GetObject(hBitmap, sizeof(BITMAP), &bmp);

    if (bmp.bmBits == false)
    {
      DeleteObject(hBitmap);
      imgH = 0;
      imgW = 0;
      return NULL;
    }

    DeleteObject(hBitmap);
    imgH = bmp.bmHeight;
    imgW = bmp.bmWidth;
    return bmp.bmBits;
#else
#ifdef _WIN32
    // Win32:
#else
    // Otherwise...
#endif
#endif
  }
}
