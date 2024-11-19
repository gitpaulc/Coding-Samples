/*  Copyright Paul Cernea, November 2024.
All Rights Reserved.*/

#include "camera.h"
#include "primitives.h"

namespace MeshRenderer
{

class Camera::Impl
{
public:
  Impl(Camera*);
  virtual ~Impl();

  Camera* pCam = nullptr;
};

Camera::Camera() : pImpl(new Impl(this))
{
}

Camera::~Camera()
{
  delete pImpl;
  pImpl = nullptr;
}

Camera::Impl::Impl(Camera* pCamera) : pCam(pCamera)
{
}

Camera::Impl::~Impl() { }

}
