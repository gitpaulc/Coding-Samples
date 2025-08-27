/*  Copyright Paul Cernea, November 2024.
All Rights Reserved.*/

#include "gl_callbacks.h"

#include "includes.h"
#include "camera.h"
#include "edge_list.h"

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
  MeshRenderer::SetWindowWidthHeight(1024);
#else
  MeshRenderer::SetWindowWidthHeight(750);
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

  std::cout << "\nNavigation: Pan Left = J, Pan Right = L, Pan Up = I, Pan Down = K, Pan Back = B, Pan Forward = Mouse Click, Zoom In = Z, Zoom Out = Y.\n";
  std::cout << "Rotation: Press R to rotate clockwise, T counter-clockwise. W, A, S, and D to rotate screen.\n";
  std::cout << "Orthogonal/Perspective: O to toggle, Depth Testing: P to toggle.\n";
  std::cout << "Wireframe/Opaque/Shading: U to toggle.\n";
  std::cout << "Use shaders: 1 to toggle, Face normals: 2 to toggle.\n";

  MeshRenderer::DoublyConnectedEdgeList::Create(filename);
  srand((unsigned)time(NULL));
  initialize_glut(&argc, argv);
  glutMainLoop();

  return 0;
}
