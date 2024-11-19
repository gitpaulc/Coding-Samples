/*  Copyright Paul Cernea, November 2024.
All Rights Reserved.*/

#ifndef MESH_CAMERA_H
#define MESH_CAMERA_H

namespace MeshRenderer
{

class Camera
{
public:
    Camera();
    virtual ~Camera();
private:
    class Impl;
    Impl* pImpl = nullptr;
};

}

#endif //def MESH_CAMERA_H
