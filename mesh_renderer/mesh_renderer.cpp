/*  Copyright Paul Cernea, November 2024.
All Rights Reserved.*/

#include "gl_callbacks.h"

#include "includes.h"
#include "camera.h"
#include "edge_list.h"
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
  std::string filename = "";
  if (argc < 2)
  {
    std::cout << "\nPlease enter a filename with .obj extension.\n-->  ";
    std::cin >> filename;
  }
  else
  {
    filename = std::string(argv[1]);
  }
  if (filename.empty())
  {
    std::cout << "\nPlease enter a filename with .obj extension.\n-->  ";
    std::cin >> filename;
  }
  if (filename.empty())
  {
    std::cout << "\nBad filename. Press any key to exit:\n-->  ";
    std::cin >> filename;
    return 0;
  }

  srand((unsigned)time(NULL));
  //initialize_glut(&argc, argv);
  //glutMainLoop();

  MeshRenderer::DoublyConnectedEdgeList::Run(filename);
  MeshRenderer::Camera cam;
  return 0;
}
