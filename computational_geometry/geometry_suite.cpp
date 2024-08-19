#include "point_3d.h"
using namespace ComputationalGeometry;

void basic_test()
{
  {
    point_3d<double> P(1.0, 2.0, 3.0);
    std::cout << "\n//////\nThe dimension is " << P.get_dimension() << ".";
    P.print("\n");
    std::cout << "\n";

    point_2d<double> Q(1.0, 2.0);
    std::cout << "\n//////\nThe dimension is " << Q.get_dimension() << ".";
    Q.print("\n");
    std::cout << "\n";
  }
  
  {
    point_3d<double> P(1.0, 2.0, 3.0);
    point_3d<double> Q(1.0, -2.0, 3.0);

    printf("%f\n", point_3d<double>::sq_distance(P,Q));
  }
  
  {
    point_2d<double> P(1.5, 2.0);
    point_2d<double> Q(1.0, -2.0);

    printf("%f\n", point_2d<double>::sq_distance(P,Q));
  }
  
  {
    std::set<point_3d<double> > A;
    point_3d<double> a(1.5, 2.0, 0);
    point_3d<double> b(1.0, 3.0, 0);
    point_3d<double> c(-1.5, 7.0, 0);
    
    std::set<point_3d<double> > B;
    point_3d<double> d(4.5, 2.3, 0);
    point_3d<double> e(-1.55, 2.6, 0);
    point_3d<double> f(88.3, 0.001, 0);
    
    A.insert(a);
    A.insert(b);
    A.insert(c);
    
    B.insert(d);
    B.insert(e);
    B.insert(f);
    
    point_3d<double> p, q;
    
    double min = point_3d<double>::naive_min_sq_distance(A, B, p, q);
    
    std::cout << "\n//////\n" << min;
    p.print("\n");
    q.print("\n");
    std::cout << "\n";
  }
  
  {
    std::set<point_2d<double> > A;
    point_2d<double> a(1.5, 2.0);
    point_2d<double> b(1.0, 3.0);
    point_2d<double> c(-1.5, 7.0);
    
    std::set<point_2d<double> > B;
    point_2d<double> d(4.5, 2.3);
    point_2d<double> e(-1.35, 2.6);
    point_2d<double> f(88.3, 0.001);
    
    A.insert(a);
    A.insert(b);
    A.insert(c);
    
    B.insert(d);
    B.insert(e);
    B.insert(f);
    
    point_2d<double> p, q;
    
    double min = point_2d<double>::naive_min_sq_distance(A, B, p, q);
    
    std::cout << "\n//////\n" << min << "\n";
    p.print("\n");
    q.print("\n");
    std::cout << "\n";
  }
  
  {
    std::set<point_2d<double> > A;
    point_2d<double> a(1.5, 2.0);
    point_2d<double> b(1.0, 3.0);
    point_2d<double> c(-1.5, 7.0);
    point_2d<double> d(4.5, 2.3);
    point_2d<double> e(-1.35, 2.6);
    point_2d<double> f(88.3, 0.001);
    
    A.insert(a);
    A.insert(b);
    A.insert(c);
    A.insert(d);
    A.insert(e);
    A.insert(f);
    
    point_2d<double> p, q;
    
    double min = point_2d<double>::naive_min_sq_distance(A, p, q);
    
    std::cout << "\n//////\n";
    std::cout << min << "\n";
    p.print();  std::cout << "\n";
    q.print();  std::cout << "\n";
    
    min = point_2d<double>::min_sq_distance(A, p, q);
    
    std::cout << "\n/!!!!/\n";
    std::cout << min << "\n";
    p.print();  std::cout << "\n";
    q.print();  std::cout << "\n";
  }
}

void mouse(int button, int state, int x, int y)
{
  if((button == GLUT_LEFT_BUTTON) && (state == GLUT_UP))
  {
    recompute();
  }
  glutPostRedisplay();
}

void render()
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glColor3f(0.0f, 0.0f, 0.0f);
  glPointSize(3.0f);

  glBegin(GL_POINTS);
  {
    int sizPointArray = PointArray().size();

    for (int i = 0; i < sizPointArray; ++i)
    {
      const point_2d<double>& P = PointArray()[i];
      glVertex2f(P.x, P.y);
      //P.print("\n");
    }
  }
  glEnd();
  
  glColor3f(1.0f, 0.0f, 0.0f);

  glBegin(GL_LINE_LOOP);
  {
    int sizPointArray = ConvexHull().size();

    for (int i = 0; i < sizPointArray; i++)
    {
      point_2d<double> P = ConvexHull()[i];
      glVertex2f(P.x, P.y);
      //P.print("\n");
    }
  }
  glEnd();

  glutSwapBuffers();
}

int main(int argc, char **argv)
{
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
