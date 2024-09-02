
#include "gl_callbacks.h"

#include "includes.h"
#include "point_cloud.h"

#ifdef USE_OPEN_CV
#include "create_video.h"
#endif // def USE_OPEN_CV

int main(int argc, char **argv)
{
#ifdef USE_OPEN_CV
  {
    std::string errorMessage = "";
    ComputationalGeometry::VideoMode vm = ComputationalGeometry::VideoMode::Usual;
    ComputationalGeometry::createVideo(errorMessage, vm);
    return 0;
  }
#endif // def USE_OPEN_CV
#ifdef __APPLE__
  ComputationalGeometry::SetWindowWidthHeight(1024);
#else
  ComputationalGeometry::SetWindowWidthHeight(750);
#endif
  if (argc == 2)
  {
    int numPoints = atoi(argv[1]);
    
    if (numPoints > 0)
    {
      ComputationalGeometry::numRandomPoints() = numPoints;
    }
  }
  
  //ComputationalGeometry::PointCloud::Get().unitTest();
  srand((unsigned)time(NULL));
  initialize_glut(&argc, argv);
  glutMainLoop();
  
  return 0;
}
