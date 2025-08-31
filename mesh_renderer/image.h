/*  Copyright Paul Cernea, August 2025.
All Rights Reserved.*/

#ifndef MESH_IMAGE_H
#define MESH_IMAGE_H

#include "includes.h"

namespace MeshRenderer
{
  void* loadImage(const std::string& filename, long& imgW, long& imgH);
}

#endif //def MESH_IMAGE_H
