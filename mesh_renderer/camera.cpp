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
  ComputationalGeometry::point3d eye;
  bool orthogonalView = false;
  ComputationalGeometry::Plane3d screen; // Near plane.
  ComputationalGeometry::Plane3d farPlane;
  bool hasFarPlane = false;
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
  using namespace ComputationalGeometry;
  screen = Plane3d(point3d(0.0, 0.0, 1.0), point3d(1.0, 0.0, 1.0), point3d(0.0, 1.0, 1.0));
  farPlane = Plane3d(point3d(0.0, 0.0, 10.0), point3d(1.0, 0.0, 10.0), point3d(0.0, 10.0, 10.0));
}

Camera::Impl::~Impl() { }

}
