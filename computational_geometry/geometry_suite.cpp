
#include "gl_callbacks.h"
#include "point_3d.h"

using namespace ComputationalGeometry;

void basic_test()
{
  {
    point_3d P(1.0, 2.0, 3.0);
    std::cout << "\n//////\nThe dimension is " << P.GetDimension() << ".";
    P.print("\n");
    std::cout << "\n";

    point_2d Q(1.0, 2.0);
    std::cout << "\n//////\nThe dimension is " << Q.GetDimension() << ".";
    Q.print("\n");
    std::cout << "\n";
  }
  
  {
    point_3d P(1.0, 2.0, 3.0);
    point_3d Q(1.0, -2.0, 3.0);

    printf("%f\n", point_3d::sq_distance(P,Q));
  }
  
  {
    point_2d P(1.5, 2.0);
    point_2d Q(1.0, -2.0);

    printf("%f\n", point_2d::sq_distance(P,Q));
  }
  
  {
    std::set<point_3d > A;
    point_3d a(1.5, 2.0, 0);
    point_3d b(1.0, 3.0, 0);
    point_3d c(-1.5, 7.0, 0);
    
    std::set<point_3d > B;
    point_3d d(4.5, 2.3, 0);
    point_3d e(-1.55, 2.6, 0);
    point_3d f(88.3, 0.001, 0);
    
    A.insert(a);
    A.insert(b);
    A.insert(c);
    
    B.insert(d);
    B.insert(e);
    B.insert(f);
    
    point_3d p, q;
    
    double min = point_3d::naive_min_sq_distance(A, B, p, q);
    
    std::cout << "\n//////\n" << min;
    p.print("\n");
    q.print("\n");
    std::cout << "\n";
  }
  
  {
    std::set<point_2d > A;
    point_2d a(1.5, 2.0);
    point_2d b(1.0, 3.0);
    point_2d c(-1.5, 7.0);
    
    std::set<point_2d > B;
    point_2d d(4.5, 2.3);
    point_2d e(-1.35, 2.6);
    point_2d f(88.3, 0.001);
    
    A.insert(a);
    A.insert(b);
    A.insert(c);
    
    B.insert(d);
    B.insert(e);
    B.insert(f);
    
    point_2d p, q;
    
    double min = point_2d::naive_min_sq_distance(A, B, p, q);
    
    std::cout << "\n//////\n" << min << "\n";
    p.print("\n");
    q.print("\n");
    std::cout << "\n";
  }
  
  {
    std::set<point_2d > A;
    point_2d a(1.5, 2.0);
    point_2d b(1.0, 3.0);
    point_2d c(-1.5, 7.0);
    point_2d d(4.5, 2.3);
    point_2d e(-1.35, 2.6);
    point_2d f(88.3, 0.001);
    
    A.insert(a);
    A.insert(b);
    A.insert(c);
    A.insert(d);
    A.insert(e);
    A.insert(f);
    
    point_2d p, q;
    
    double min = point_2d::naive_min_sq_distance(A, p, q);
    
    std::cout << "\n//////\n";
    std::cout << min << "\n";
    p.print();  std::cout << "\n";
    q.print();  std::cout << "\n";
    
    min = point_2d::min_sq_distance(A, p, q);
    
    std::cout << "\n/!!!!/\n";
    std::cout << min << "\n";
    p.print();  std::cout << "\n";
    q.print();  std::cout << "\n";
  }
}

void render()
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glColor3f(0.0f, 0.0f, 0.0f);
  glPointSize(3.0f);

  glBegin(GL_POINTS);
  {
    int sizPointArray = point_2d::PointArray().size();

    for (int i = 0; i < sizPointArray; ++i)
    {
      const point_2d& P = point_2d::PointArray()[i];
      glVertex2f(P.x, P.y);
      //P.print("\n");
    }
  }
  glEnd();
  
  glColor3f(1.0f, 0.0f, 0.0f);

  glBegin(GL_LINE_LOOP);
  {
    int sizPointArray = point_2d::ConvexHull().size();

    for (int i = 0; i < sizPointArray; i++)
    {
      point_2d P = point_2d::ConvexHull()[i];
      glVertex2f(P.x, P.y);
      //P.print("\n");
    }
  }
  glEnd();

  glutSwapBuffers();
}

int main(int argc, char **argv)
{
  SetWindowWidthHeight(1024);
  if (argc == 2)
  {
    int numPoints = atoi(argv[1]);
    
    if (numPoints > 0)
    {
      numRandomPoints() = numPoints;
    }
  }
  
  //basic_test();
  srand(time(NULL));
  initialize_glut(&argc, argv);
  glutMainLoop();
  
  return 0;
}
