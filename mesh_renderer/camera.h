/*  Copyright Paul Cernea, November 2024.
All Rights Reserved.*/

#ifndef MESH_CAMERA_H
#define MESH_CAMERA_H

namespace ComputationalGeometry
{
  class Matrix2d; // Forward declaration.
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
  /** \brief Point on the screen. Eye looks straight ahead at screen. */
  ComputationalGeometry::point3d getEyeCast() const;
  ComputationalGeometry::Plane3d getNearPlane() const;
  /** \brief Same as near plane. */
  ComputationalGeometry::Plane3d getScreen() const;
  /** \brief Interprets screen as oriented plane instead of just plane. */
  double getScreenAxesRotation() const;
  ComputationalGeometry::Matrix2d getScreenAxesRotMatrix() const;
  bool hasFarPlane() const;
  bool viewIsOrthogonal() const;
  /** \brief Interprets screen as oriented plane instead of just plane. */
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
