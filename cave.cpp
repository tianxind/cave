#include <OpenMesh/Core/IO/Options.hh>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <Eigen/Core>
#include "mesh_definitions.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <GLUT/glut.h>

using namespace Eigen;
using namespace std;
using namespace OpenMesh;

static const double pi = 3.14159265359;
//static int kMsecsPerFrame = 100;
//static struct timeval last_idle_time;

bool leftDown = false, rightDown = false, middleDown = false;
int lastPos[2];
float cameraPos[4] = {0,0,4,1};
GLfloat light_position[] = {1, 1, 1, 0};
float shininess[] = {50.0};
float specular[] = {1.0, 1.0, 1.0, 1.0};

// Number of vertices and faces of the mesh 
int n_vertices = 0, n_faces = 0;

Vec3f up, pan;
bool showAxes = true;
int windowWidth = 640, windowHeight = 480;

void drawSurface() {
  // Sixteen control points (substitute these values with your own if you like)
  double Ax = -2.0; double Ay =  2.0; double Az =  1.0;
  double Bx = -1.0; double By =  3.0; double Bz = -1.0;
  double Cx =  1.0; double Cy =  3.0; double Cz =  1.0;
  double Dx =  2.0; double Dy =  2.0; double Dz = -1.0;

  double Ex = -1.5; double Ey =  1.0; double Ez =  1.0;
  double Fx = -0.5; double Fy =  1.5; double Fz = -1.0;
  double Gx =  1.5; double Gy =  1.5; double Gz =  1.0;
  double Hx =  2.5; double Hy =  1.0; double Hz = -1.0;

  double Ix = -2.5; double Iy = -1.0; double Iz =  1.0;
  double Jx = -1.5; double Jy = -0.5; double Jz = -1.0;
  double Kx =  0.5; double Ky = -0.5; double Kz =  1.0;
  double Lx =  1.5; double Ly = -1.0; double Lz = -1.0;

  double Mx = -2.0; double My = -2.0; double Mz =  1.0;
  double Nx = -1.0; double Ny = -1.0; double Nz = -1.0;
  double Ox =  1.0; double Oy = -1.0; double Oz =  1.0;
  double Px =  2.0; double Py = -2.0; double Pz = -1.0;

// Vertices (maximum detail will now be 20·20 quads)
double Xv[21][21];
double Yv[21][21];
double Zv[21][21];

// Normal vectors
double Xn[21][21];
double Yn[21][21];
double Zn[21][21];


  /* We will not actually draw a curved surface, but we will divide the
     surface into small quads and draw them. If the quads are small enough,
     it will appear as a curved surface. We will use a variable, detail, to
     define how many quads to use. Since the variables goes from 1.0 to 0.0
     we must change them by 1/detail from vertex to vertex. We will also
     store the vertices and the normal vectors in arrays and draw them in a
     separate loop */

  // Detail of 10 mean that we will calculate 11·11 vertices
  int    detail = 10;
  double change = 1.0 / (double)detail;
 
  // Just making sure that the detail level is not set too high
  if(detail > 20)
    {
      detail = 20;
    }
  
  // Variables
  double a = 1.0;
  double b = 1.0 - a;
  double c = 1.0;
  double d = 1.0 - c;
  
  // Tangent vectors
  double Xta;
  double Yta;
  double Zta;
  
  double Xtc;
  double Ytc;
  double Ztc;

  /* Since we have two variables, we need two loops, we will change the
     a-variable from 1.0 to 0.0 by steps of 1/detail ( = change), and for each
     step we loop the c-variable from 1.0 to 0.0, thus creating a grid of
     points covering the surface. Note that we could have had separate detail
     levels for the a-variable and the c-variable if we wanted to */
  for(int i = 0; i <= detail; i++)
    {
      for(int j = 0; j <= detail; j++)
	{
	  // First get the vertices
	  Xv[i][j] = Ax*a*a*a*c*c*c   + Bx*3*a*a*a*c*c*d
	    + Cx*3*a*a*a*c*d*d + Dx*a*a*a*d*d*d
	    + Ex*3*a*a*b*c*c*c + Fx*9*a*a*b*c*c*d
	    + Gx*9*a*a*b*c*d*d + Hx*3*a*a*b*d*d*d
	    + Ix*3*a*b*b*c*c*c + Jx*9*a*b*b*c*c*d
	    + Kx*9*a*b*b*c*d*d + Lx*3*a*b*b*d*d*d
	    + Mx*b*b*b*c*c*c   + Nx*3*b*b*b*c*c*d
	    + Ox*3*b*b*b*c*d*d + Px*b*b*b*d*d*d;
	  
	  Yv[i][j] = Ay*a*a*a*c*c*c   + By*3*a*a*a*c*c*d
	    + Cy*3*a*a*a*c*d*d + Dy*a*a*a*d*d*d
	    + Ey*3*a*a*b*c*c*c + Fy*9*a*a*b*c*c*d
	    + Gy*9*a*a*b*c*d*d + Hy*3*a*a*b*d*d*d
	    + Iy*3*a*b*b*c*c*c + Jy*9*a*b*b*c*c*d
	    + Ky*9*a*b*b*c*d*d + Ly*3*a*b*b*d*d*d
	    + My*b*b*b*c*c*c   + Ny*3*b*b*b*c*c*d
	    + Oy*3*b*b*b*c*d*d + Py*b*b*b*d*d*d;
	  
	  Zv[i][j] = Az*a*a*a*c*c*c   + Bz*3*a*a*a*c*c*d
	    + Cz*3*a*a*a*c*d*d + Dz*a*a*a*d*d*d
	    + Ez*3*a*a*b*c*c*c + Fz*9*a*a*b*c*c*d
	    + Gz*9*a*a*b*c*d*d + Hz*3*a*a*b*d*d*d
	    + Iz*3*a*b*b*c*c*c + Jz*9*a*b*b*c*c*d
	    + Kz*9*a*b*b*c*d*d + Lz*3*a*b*b*d*d*d
	    + Mz*b*b*b*c*c*c   + Nz*3*b*b*b*c*c*d
	    + Oz*3*b*b*b*c*d*d + Pz*b*b*b*d*d*d;
	  
	  // Then use the derived functions to get the tangent vectors
	  Xta = Ax*3*a*a*c*c*c       + Bx*9*a*a*c*c*d
	    + Cx*9*a*a*c*d*d       + Dx*3*a*a*d*d*d
	    + Ex*3*(2*a-3*a*a)*c*c*c   + Fx*9*(2*a-3*a*a)*c*c*d
	    + Gx*9*(2*a-3*a*a)*c*d*d   + Hx*3*(2*a-3*a*a)*d*d*d
	    + Ix*3*(1-4*a+3*a*a)*c*c*c + Jx*9*(1-4*a+3*a*a)*c*c*d
	    + Kx*9*(1-4*a+3*a*a)*c*d*d + Lx*3*(1-4*a+3*a*a)*d*d*d
	    + Mx*3*(2*a-1-a*a)*c*c*c   + Nx*9*(2*a-1-a*a)*c*c*d
	    + Ox*9*(2*a-1-a*a)*c*d*d   + Px*3*(2*a-1-a*a)*d*d*d;

	  Yta = Ay*3*a*a*c*c*c       + By*9*a*a*c*c*d
	    + Cy*9*a*a*c*d*d       + Dy*3*a*a*d*d*d
	    + Ey*3*(2*a-3*a*a)*c*c*c   + Fy*9*(2*a-3*a*a)*c*c*d
	    + Gy*9*(2*a-3*a*a)*c*d*d   + Hy*3*(2*a-3*a*a)*d*d*d
	    + Iy*3*(1-4*a+3*a*a)*c*c*c + Jy*9*(1-4*a+3*a*a)*c*c*d
	    + Ky*9*(1-4*a+3*a*a)*c*d*d + Ly*3*(1-4*a+3*a*a)*d*d*d
	    + My*3*(2*a-1-a*a)*c*c*c   + Ny*9*(2*a-1-a*a)*c*c*d
	    + Oy*9*(2*a-1-a*a)*c*d*d   + Py*3*(2*a-1-a*a)*d*d*d;

	  Zta = Az*3*a*a*c*c*c       + Bz*9*a*a*c*c*d
	    + Cz*9*a*a*c*d*d       + Dz*3*a*a*d*d*d
	    + Ez*3*(2*a-3*a*a)*c*c*c   + Fz*9*(2*a-3*a*a)*c*c*d
	    + Gz*9*(2*a-3*a*a)*c*d*d   + Hz*3*(2*a-3*a*a)*d*d*d
	    + Iz*3*(1-4*a+3*a*a)*c*c*c + Jz*9*(1-4*a+3*a*a)*c*c*d
	    + Kz*9*(1-4*a+3*a*a)*c*d*d + Lz*3*(1-4*a+3*a*a)*d*d*d
	    + Mz*3*(2*a-1-a*a)*c*c*c   + Nz*9*(2*a-1-a*a)*c*c*d
	    + Oz*9*(2*a-1-a*a)*c*d*d   + Pz*3*(2*a-1-a*a)*d*d*d;
	  
	  Xtc = Ax*3*a*a*a*c*c       + Bx*3*a*a*a*(2*c-3*c*c)
	    + Cx*3*a*a*a*(1-4*c+3*c*c) + Dx*3*a*a*a*(-1+2*c-c*c)
	    + Ex*9*a*a*b*c*c       + Fx*9*a*a*b*(2*c-3*c*c)
	    + Gx*9*a*a*b*(1-4*c+3*c*c) + Hx*9*a*a*b*(-1+2*c-c*c)
	    + Ix*9*a*b*b*c*c       + Jx*9*a*b*b*(2*c-3*c*c)
	    + Kx*9*a*b*b*(1-4*c+3*c*c) + Lx*9*a*b*b*(-1+2*c-c*c)
	    + Mx*3*b*b*b*c*c       + Nx*3*b*b*b*(2*c-3*c*c)
	    + Ox*3*b*b*b*(1-4*c+3*c*c) + Px*3*b*b*b*(-1+2*c-c*c);
	  
	  Ytc = Ay*3*a*a*a*c*c       + By*3*a*a*a*(2*c-3*c*c)
	    + Cy*3*a*a*a*(1-4*c+3*c*c) + Dy*3*a*a*a*(-1+2*c-c*c)
	    + Ey*9*a*a*b*c*c       + Fy*9*a*a*b*(2*c-3*c*c)
	    + Gy*9*a*a*b*(1-4*c+3*c*c) + Hy*9*a*a*b*(-1+2*c-c*c)
	    + Iy*9*a*b*b*c*c       + Jy*9*a*b*b*(2*c-3*c*c)
	    + Ky*9*a*b*b*(1-4*c+3*c*c) + Ly*9*a*b*b*(-1+2*c-c*c)
	    + My*3*b*b*b*c*c       + Ny*3*b*b*b*(2*c-3*c*c)
	    + Oy*3*b*b*b*(1-4*c+3*c*c) + Py*3*b*b*b*(-1+2*c-c*c);

	  Ztc = Az*3*a*a*a*c*c       + Bz*3*a*a*a*(2*c-3*c*c)
	    + Cz*3*a*a*a*(1-4*c+3*c*c) + Dz*3*a*a*a*(-1+2*c-c*c)
	    + Ez*9*a*a*b*c*c       + Fz*9*a*a*b*(2*c-3*c*c)
	    + Gz*9*a*a*b*(1-4*c+3*c*c) + Hz*9*a*a*b*(-1+2*c-c*c)
	    + Iz*9*a*b*b*c*c       + Jz*9*a*b*b*(2*c-3*c*c)
	    + Kz*9*a*b*b*(1-4*c+3*c*c) + Lz*9*a*b*b*(-1+2*c-c*c)
	    + Mz*3*b*b*b*c*c       + Nz*3*b*b*b*(2*c-3*c*c)
	    + Oz*3*b*b*b*(1-4*c+3*c*c) + Pz*3*b*b*b*(-1+2*c-c*c);
	  
	  // Cross the tangent vectors, put the result to the normal vector array
	  // Note: I simplified -((Xta*Ztc)-(Xtc*Zta)) to (Xtc*Zta) - (Xta*Ztc)
	  Xn[i][j] = (Yta*Ztc) - (Ytc*Zta);
	  Yn[i][j] = (Xtc*Zta) - (Xta*Ztc);
	  Zn[i][j] = (Xta*Ytc) - (Xtc*Yta);

	  // Find length of normal vector
	  double length = sqrt((Xn[i][j]*Xn[i][j])+(Yn[i][j]
						    *Yn[i][j])+(Zn[i][j]*Zn[i][j]));

	  // Normalize (and prevent divide by zero error)
	  if(length > 0)
	    {
	      length = 1.0/length;
	      Xn[i][j] *= length;
	      Yn[i][j] *= length;
	      Zn[i][j] *= length;
	    }

	  //change the c-variable within the inner loop
	  c -= change;
	  d  = 1.0 - c;
	}
      //change the a-variable outside the inner loop
      a -= change;
      b  = 1.0 - a;

      // Reset the c-variable to make it ready for the inner loop again
      c = 1.0;
      d = 1.0 - c;
    }
  /* Now we have two arrays, one with vertices, and one with normal vectors,
     drawing them is straightforward if you know how to use a graphics API.
     Following is one way to do it using openGL and triangle strips. (assuming
     GL_LIGHTING etc.. has been properly set up) */
  for(int m = 0; m < detail; m++)
    {
      glBegin(GL_TRIANGLE_STRIP);
      for(int n = 0; n <= detail; n++)
	{
	  glNormal3d(Xn[m][n],Yn[m][n],Zn[m][n]);
	  glVertex3d(Xv[m][n],Yv[m][n],Zv[m][n]);
	  
	  // Note that I used real-less-than in the first loop, since I want to
	  // access the m+1 entry in the array to properly draw the triangle strip
	  glNormal3d(Xn[m+1][n],Yn[m+1][n],Zn[m+1][n]);
	  glVertex3d(Xv[m+1][n],Yv[m+1][n],Zv[m+1][n]);
	}
      glEnd();
    }
}

void init() {
  // Init lighting
  GLfloat white[] = {1.0, 1.0, 1.0, 1.0};
  GLfloat l_ambient[] = {0.2, 0.2, 0.2, 1.0};

  glShadeModel(GL_SMOOTH);
  glLightfv(GL_LIGHT0, GL_POSITION, light_position);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, white);
  glLightfv(GL_LIGHT0, GL_SPECULAR, white);
  glLightModelfv(GL_LIGHT_MODEL_AMBIENT, l_ambient);

  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
  glEnable(GL_DEPTH_TEST);

  // set material
  glEnable(GL_COLOR_MATERIAL);
  glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
  glMaterialfv(GL_FRONT, GL_SPECULAR, specular);
  glMaterialfv(GL_FRONT, GL_SHININESS, shininess);
}

void display() {
  glClearColor(1,1,1,1);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glViewport(0,0,windowWidth,windowHeight);
  
  float ratio = (float)windowWidth / (float)windowHeight;
  gluPerspective(50, ratio, 1, 1000); // 50 degree vertical viewing angle, zNear = 1, zFar = 1000
  
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  gluLookAt(cameraPos[0]+pan[0], cameraPos[1]+pan[1], cameraPos[2]+pan[2],
	    pan[0], pan[1], pan[2], up[0], up[1], up[2]);

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
 
  if (showAxes) {
    glBegin(GL_LINES);
    glLineWidth(1);
    glColor3f(1,0,0); glVertex3f(0,0,0); glVertex3f(1,0,0); // x axis
    glColor3f(0,1,0); glVertex3f(0,0,0); glVertex3f(0,1,0); // y axis
    glColor3f(0,0,1); glVertex3f(0,0,0); glVertex3f(0,0,1); // z axis
    glEnd();
  }
  drawSurface();
  glutSwapBuffers();
}

void mouse(int button, int state, int x, int y) {
  if (button == GLUT_LEFT_BUTTON) leftDown = (state == GLUT_DOWN);
  else if (button == GLUT_RIGHT_BUTTON) rightDown = (state == GLUT_DOWN);
  else if (button == GLUT_MIDDLE_BUTTON) middleDown = (state == GLUT_DOWN);
	
  lastPos[0] = x;
  lastPos[1] = y;
}

void mouseMoved(int x, int y) {
  int dx = x - lastPos[0];
  int dy = y - lastPos[1];
  Vec3f curCamera(cameraPos[0],cameraPos[1],cameraPos[2]);
  Vec3f curCameraNormalized = curCamera.normalized();
  Vec3f right = up % curCameraNormalized;

  if (leftDown) {
    // Assume here that up vector is (0,1,0)
    Vec3f newPos = curCamera - 2 * ((float)dx/(float)windowWidth) * right
      + 2 * ((float)dy/(float)windowHeight) * up;
    newPos = newPos.normalized() * curCamera.length();
    
    up = up - (up | newPos) * newPos / newPos.sqrnorm();
    up.normalize();
		
    for (int i = 0; i < 3; i++) cameraPos[i] = newPos[i];
  }
  else if (rightDown) for (int i = 0; i < 3; i++) cameraPos[i] /= pow(1.1,dy*.1);
  else if (middleDown) {
    pan = -2*(float)((float)dx/(float)windowWidth) * right + 2*(float)((float)dy/(float)windowHeight) * up;
  }
  
  lastPos[0] = x;
  lastPos[1] = y;
  
  glutPostRedisplay();
}

void keyboard(unsigned char key, int x, int y) {
  if (key == 'q' || key == 'Q') exit(0);
  else if (key == 'a' || key == 'A') showAxes = !showAxes;
}

void reshape(int width, int height) {
  windowWidth = width;
  windowHeight = height;
  glutPostRedisplay();
}

int main(int argc, char** argv) {
  up = Vec3f(0, 1, 0);
  pan = Vec3f(0, 0, 0);
  Vec3f center = Vec3f(0, 0, 0);

  glutInit(&argc, argv); 
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH); 
  glutInitWindowSize(windowWidth, windowHeight); 
  glutCreateWindow(argv[0]);
  
  init();

  glutDisplayFunc(display);
  glutMotionFunc(mouseMoved);
  glutMouseFunc(mouse);
  glutReshapeFunc(reshape);
  glutKeyboardFunc(keyboard);

  glutMainLoop();
  
  return 0;
}
