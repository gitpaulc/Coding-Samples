/*  Copyright Paul Cernea, November 2024.
All Rights Reserved.*/

#include "camera.h"
#include "edge_list.h"
#include "primitives.h"

namespace MeshRenderer
{
static int gWindowWidth = 1024;
static int gWindowHeight = 1024;

static ComputationalGeometry::point2d gWindowMin(-1, -1);
static ComputationalGeometry::point2d gWindowMax(1, 1);

void SetWindowWidthHeight(int ww, int hh)
{
  gWindowWidth = ww;
  if (hh < 0) { hh = ww; }
  gWindowHeight = hh;
}

void GetWindowWidthHeight(int& ww, int& hh)
{
  ww = gWindowWidth;
  hh = gWindowHeight;
}


class Camera::Impl
{
public:
  Impl(Camera*);
  Impl(Camera*, const DoublyConnectedEdgeList& mesh);
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

Camera::Camera(const DoublyConnectedEdgeList& mesh) : pImpl(new Impl(this, mesh))
{
}

Camera::~Camera()
{
  delete pImpl;
  pImpl = nullptr;
}

ComputationalGeometry::point3d Camera::getEye() const
{
  if (pImpl == nullptr) { return ComputationalGeometry::point3d(); }
  return pImpl->eye;
}

void Camera::setEye(const ComputationalGeometry::point3d& eyeIn)
{
  if (pImpl == nullptr) { return; }
  pImpl->eye = eyeIn;
}

ComputationalGeometry::Plane3d Camera::getFarPlane() const
{
  if (pImpl == nullptr) { return ComputationalGeometry::Plane3d(); }
  return pImpl->farPlane;
}

ComputationalGeometry::Plane3d Camera::getNearPlane() const
{
  if (pImpl == nullptr) { return ComputationalGeometry::Plane3d(); }
  return pImpl->screen;
}

ComputationalGeometry::Plane3d Camera::getScreen() const
{
  if (pImpl == nullptr) { return ComputationalGeometry::Plane3d(); }
  return pImpl->screen;
}

void Camera::setScreen(const ComputationalGeometry::Plane3d& scr)
{
  if (pImpl == nullptr) { return; }
  pImpl->screen = scr;
}

bool Camera::hasFarPlane() const
{
  if (pImpl == nullptr) { return false; }
  return pImpl->hasFarPlane;
}

bool Camera::viewIsOrthogonal() const
{
  if (pImpl == nullptr) { return false; }
  return pImpl->orthogonalView;
}

Camera::Impl::Impl(Camera* pCamera) : pCam(pCamera)
{
  using namespace ComputationalGeometry;
  screen = Plane3d(point3d(0.0, 0.0, 1.0), point3d(1.0, 0.0, 1.0), point3d(0.0, 1.0, 1.0));
  farPlane = Plane3d(point3d(0.0, 0.0, 10.0), point3d(1.0, 0.0, 10.0), point3d(0.0, 10.0, 10.0));
}

Camera::Impl::Impl(Camera* pCamera, const DoublyConnectedEdgeList& mesh) : pCam(pCamera)
{
  using namespace ComputationalGeometry;
  point3d boxMax, boxMin, center;
  mesh.getBoundingBox(boxMax, boxMin);
  center.x = (boxMax.x + boxMin.x) * 0.5;
  center.y = (boxMax.y + boxMin.y) * 0.5;
  center.z = (boxMax.z + boxMin.z) * 0.5;
  auto diffZ = (boxMax.z - center.z) * 0.25;
  auto screenZ = boxMin.z - diffZ;
  auto farPlaneZ = boxMax.z + diffZ;
  screen = Plane3d(point3d(center.x, center.y, screenZ), point3d(center.x + 1.0, center.y, screenZ), point3d(center.x, center.y + 1.0, screenZ));
  farPlane = Plane3d(point3d(center.x, center.y, farPlaneZ), point3d(center.x + 1.0, center.y, farPlaneZ), point3d(center.x, center.y + 1.0, farPlaneZ));
}

Camera::Impl::~Impl() { }

}
