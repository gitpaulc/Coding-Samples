
#include "gl_callbacks.h"

#include "includes.h"
#include "point_cloud.h"

int main(int argc, char **argv)
{
  ComputationalGeometry::SetWindowWidthHeight(1024);
  if (argc == 2)
  {
    int numPoints = atoi(argv[1]);
    
    if (numPoints > 0)
    {
      ComputationalGeometry::numRandomPoints() = numPoints;
    }
  }
  
  //ComputationalGeometry::PointCloud::Get().unitTest();
  srand(time(NULL));
  initialize_glut(&argc, argv);
  glutMainLoop();
  
  return 0;
}
