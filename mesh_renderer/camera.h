/*  Copyright Paul Cernea, November 2024.
All Rights Reserved.*/

#ifndef MESH_CAMERA_H
#define MESH_CAMERA_H

namespace ComputationalGeometry
{
  class point3d; // Forward declaration.
  class Plane3d; // Forward declaration.
}

namespace MeshRenderer
{
class DoublyConnectedEdgeList; // Mesh.

void SetWindowWidthHeight(int ww, int hh = -1);
void GetWindowWidthHeight(int& ww, int& hh);

class Camera
{
  ComputationalGeometry::Plane3d getFarPlane() const;
public:
  Camera();
  Camera(const DoublyConnectedEdgeList& mesh);
  virtual ~Camera();
  ComputationalGeometry::point3d getEye() const;
  void setEye(const ComputationalGeometry::point3d&);
  ComputationalGeometry::Plane3d getNearPlane() const;
  /** \brief Same as near plane. */
  ComputationalGeometry::Plane3d getScreen() const;
  double getScreenAxesRotation() const;
  bool hasFarPlane() const;
  bool viewIsOrthogonal() const;
  void setScreenAxesRotation(double theta);
  /** \brief Same as set near plane. */
  void setScreen(const ComputationalGeometry::Plane3d&);
  void setViewOrthogonal(bool);

private:
  class Impl;
  Impl* pImpl = nullptr;
};

}

#endif //def MESH_CAMERA_H
