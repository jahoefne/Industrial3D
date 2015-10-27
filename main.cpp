
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <GL/glew.h>
#include <GL/glut.h>
#include <trackball.h>

/*
 * (c) Copyright 1993, 1994, Silicon Graphics, Inc.
 * ALL RIGHTS RESERVED
 * Permission to use, copy, modify, and distribute this software for
 * any purpose and without fee is hereby granted, provided that the above
 * copyright notice appear in all copies and that both the copyright notice
 * and this permission notice appear in supporting documentation, and that
 * the name of Silicon Graphics, Inc. not be used in advertising
 * or publicity pertaining to distribution of the software without specific,
 * written prior permission.
 *
 * THE MATERIAL EMBODIED ON THIS SOFTWARE IS PROVIDED TO YOU "AS-IS"
 * AND WITHOUT WARRANTY OF ANY KIND, EXPRESS, IMPLIED OR OTHERWISE,
 * INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY OR
 * FITNESS FOR A PARTICULAR PURPOSE.  IN NO EVENT SHALL SILICON
 * GRAPHICS, INC.  BE LIABLE TO YOU OR ANYONE ELSE FOR ANY DIRECT,
 * SPECIAL, INCIDENTAL, INDIRECT OR CONSEQUENTIAL DAMAGES OF ANY
 * KIND, OR ANY DAMAGES WHATSOEVER, INCLUDING WITHOUT LIMITATION,
 * LOSS OF PROFIT, LOSS OF USE, SAVINGS OR REVENUE, OR THE CLAIMS OF
 * THIRD PARTIES, WHETHER OR NOT SILICON GRAPHICS, INC.  HAS BEEN
 * ADVISED OF THE POSSIBILITY OF SUCH LOSS, HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, ARISING OUT OF OR IN CONNECTION WITH THE
 * POSSESSION, USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 * US Government Users Restricted Rights
 * Use, duplication, or disclosure by the Government is subject to
 * restrictions set forth in FAR 52.227.19(c)(2) or subparagraph
 * (c)(1)(ii) of the Rights in Technical Data and Computer Software
 * clause at DFARS 252.227-7013 and/or in similar or successor
 * clauses in the FAR or the DOD or NASA FAR Supplement.
 * Unpublished-- rights reserved under the copyright laws of the
 * United States.  Contractor/manufacturer is Silicon Graphics,
 * Inc., 2011 N.  Shoreline Blvd., Mountain View, CA 94039-7311.
 *
 * OpenGL(TM) is a trademark of Silicon Graphics, Inc.
 */
/*
 * Trackball code:
 *
 * Implementation of a virtual trackball.
 * Implemented by Gavin Bell, lots of ideas from Thant Tessman and
 *   the August '88 issue of Siggraph's "Computer Graphics," pp. 121-129.
 *
 * Vector manip code:
 *
 * Original code from:
 * David M. Ciemiewicz, Mark Grossman, Henry Moreton, and Paul Haeberli
 *
 * Much mucking with by:
 * Gavin Bell
 */
#if defined(_WIN32)
#pragma warning (disable:4244)          /* disable bogus conversion warnings */
#endif
#include <math.h>
#include "trackball.h"

/*
 * This size should really be based on the distance from the center of
 * rotation to the point on the object underneath the mouse.  That
 * point would then track the mouse as closely as possible.  This is a
 * simple example, though, so that is left as an Exercise for the
 * Programmer.
 */
#define TRACKBALLSIZE  (0.8f)

/*
 * Local function prototypes (not defined in trackball.h)
 */

///////////////////////////////////////////////////////////////
//<---------------------------Start Load Data----------------------->>
///////////////////////////////////////////////////////////////
#define MaxVertices 400000
#define MaxFaces    400000 
#define MaxGroups   100

float       vertex[MaxVertices*3];
unsigned int  face[MaxFaces*3];
char    group_name[MaxGroups][80];
int     start_face[MaxGroups];

int vertices = 0;
int faces    = 0;
int groups   = 0;

void read_wavefront(const char *filename)
{
  char line[80];
  FILE *f = fopen(filename, "r");
  while(fgets(line, sizeof(line), f))
    switch(line[0])
      {
      case 'v':
        sscanf(&line[1],  "%f %f %f", &vertex[vertices*3],
               &vertex[vertices*3+1], &vertex[vertices*3+2]);
        ++vertices;
        break;
      case 'g':
        sscanf(&line[1], "%s", group_name[groups]);
        start_face[groups++] = faces;
        break;
      case 'f':
        sscanf(&line[1],  "%d %d %d", &face[faces*3],
               &face[faces*3+1], &face[faces*3+2]);
        --face[faces*3]; --face[faces*3+1];
        --face[faces*3+2]; ++faces; 
        break;
      }
  fclose(f);
  start_face[groups] = faces;
  printf("Read %d vertices and %d faces within %d groups from %s\n",
         vertices, faces, groups, filename);
}

void write_wavefront(int group_number)
{
  int i = 0; char n[80], *p = group_name[group_number];
  while (*p != '%' && *p != '\0') n[i++] = *p++; // remove % from name
  n[i++] = '.'; n[i++] = 'o'; n[i++] = 'b'; n[i++] = 'j'; n[i] = '\0';
  FILE *f = fopen(n, "w"); fprintf(f, "# Wavefront OBJ file\n");
  for (i = 0; i < vertices; i++)
    fprintf(f, "v %g %g %g\n", vertex[i*3], vertex[i*3+1], vertex[i*3+2]);
  fprintf(f, "g %s\n", group_name[group_number]);
  for (i = start_face[group_number]; i < start_face[group_number+1]; ++i)
    fprintf(f, "f %d %d %d\n", face[i*3]+1, face[i*3+1]+1, face[i*3+2]+1);
  fclose(f);
}


///////////////////////////////////////////////////////////////
//<---------------------------End Load Data----------------------->>
///////////////////////////////////////////////////////////////
static float tb_project_to_sphere(float, float, float);
static void normalize_quat(float [4]);

void vzero(float *v)
{
    v[0] = 0.0;
    v[1] = 0.0;
    v[2] = 0.0;
}

void vset(float *v, float x, float y, float z)
{
    v[0] = x;
    v[1] = y;
    v[2] = z;
}

void vsub(const float *src1, const float *src2, float *dst)
{
    dst[0] = src1[0] - src2[0];
    dst[1] = src1[1] - src2[1];
    dst[2] = src1[2] - src2[2];
}

void vcopy(const float *v1, float *v2)
{
    register int i;
    for (i = 0 ; i < 3 ; i++)
        v2[i] = v1[i];
}

void vcross(const float *v1, const float *v2, float *cross)
{
    float temp[3];

    temp[0] = (v1[1] * v2[2]) - (v1[2] * v2[1]);
    temp[1] = (v1[2] * v2[0]) - (v1[0] * v2[2]);
    temp[2] = (v1[0] * v2[1]) - (v1[1] * v2[0]);
    vcopy(temp, cross);
}

float vlength(const float *v)
{
    return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

void vscale(float *v, float div)
{
    v[0] *= div;
    v[1] *= div;
    v[2] *= div;
}

void vnormal(float *v)
{
    vscale(v,1.0/vlength(v));
}

float vdot(const float *v1, const float *v2)
{
    return v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2];
}

void vadd(const float *src1, const float *src2, float *dst)
{
    dst[0] = src1[0] + src2[0];
    dst[1] = src1[1] + src2[1];
    dst[2] = src1[2] + src2[2];
}

/*
 * Ok, simulate a track-ball.  Project the points onto the virtual
 * trackball, then figure out the axis of rotation, which is the cross
 * product of P1 P2 and O P1 (O is the center of the ball, 0,0,0)
 * Note:  This is a deformed trackball-- is a trackball in the center,
 * but is deformed into a hyperbolic sheet of rotation away from the
 * center.  This particular function was chosen after trying out
 * several variations.
 *
 * It is assumed that the arguments to this routine are in the range
 * (-1.0 ... 1.0)
 */
void trackball(float q[4], float p1x, float p1y, float p2x, float p2y)
{
    float a[3]; /* Axis of rotation */
    float phi;  /* how much to rotate about axis */
    float p1[3], p2[3], d[3];
    float t;

    if (p1x == p2x && p1y == p2y) {
        /* Zero rotation */
        vzero(q);
        q[3] = 1.0;
        return;
    }

    /*
     * First, figure out z-coordinates for projection of P1 and P2 to
     * deformed sphere
     */
    vset(p1,p1x,p1y,tb_project_to_sphere(TRACKBALLSIZE,p1x,p1y));
    vset(p2,p2x,p2y,tb_project_to_sphere(TRACKBALLSIZE,p2x,p2y));

    /*
     *  Now, we want the cross product of P1 and P2
     */
    vcross(p2,p1,a);

    /*
     *  Figure out how much to rotate around that axis.
     */
    vsub(p1,p2,d);
    t = vlength(d) / (2.0*TRACKBALLSIZE);

    /*
     * Avoid problems with out-of-control values...
     */
    if (t > 1.0) t = 1.0;
    if (t < -1.0) t = -1.0;
    phi = 2.0 * asin(t);

    axis_to_quat(a,phi,q);
}

/*
 *  Given an axis and angle, compute quaternion.
 */
void axis_to_quat(float a[3], float phi, float q[4])
{
    vnormal(a);
    vcopy(a,q);
    vscale(q,sin(phi/2.0));
    q[3] = cos(phi/2.0);
}

/*
 * Project an x,y pair onto a sphere of radius r OR a hyperbolic sheet
 * if we are away from the center of the sphere.
 */
static float tb_project_to_sphere(float r, float x, float y)
{
    float d, t, z;

    d = sqrt(x*x + y*y);
    if (d < r * 0.70710678118654752440) {    /* Inside sphere */
        z = sqrt(r*r - d*d);
    } else {           /* On hyperbola */
        t = r / 1.41421356237309504880;
        z = t*t / d;
    }
    return z;
}

/*
 * Given two rotations, e1 and e2, expressed as quaternion rotations,
 * figure out the equivalent single rotation and stuff it into dest.
 *
 * This routine also normalizes the result every RENORMCOUNT times it is
 * called, to keep error from creeping in.
 *
 * NOTE: This routine is written so that q1 or q2 may be the same
 * as dest (or each other).
 */

#define RENORMCOUNT 97

void add_quats(float q1[4], float q2[4], float dest[4])
{
    static int count=0;
    float t1[4], t2[4], t3[4];
    float tf[4];

#if 0
printf("q1 = %f %f %f %f\n", q1[0], q1[1], q1[2], q1[3]);
printf("q2 = %f %f %f %f\n", q2[0], q2[1], q2[2], q2[3]);
#endif

    vcopy(q1,t1);
    vscale(t1,q2[3]);

    vcopy(q2,t2);
    vscale(t2,q1[3]);

    vcross(q2,q1,t3);
    vadd(t1,t2,tf);
    vadd(t3,tf,tf);
    tf[3] = q1[3] * q2[3] - vdot(q1,q2);

#if 0
printf("tf = %f %f %f %f\n", tf[0], tf[1], tf[2], tf[3]);
#endif

    dest[0] = tf[0];
    dest[1] = tf[1];
    dest[2] = tf[2];
    dest[3] = tf[3];

    if (++count > RENORMCOUNT) {
        count = 0;
        normalize_quat(dest);
    }
}

/*
 * Quaternions always obey:  a^2 + b^2 + c^2 + d^2 = 1.0
 * If they don't add up to 1.0, dividing by their magnitued will
 * renormalize them.
 *
 * Note: See the following for more information on quaternions:
 *
 * - Shoemake, K., Animating rotation with quaternion curves, Computer
 *   Graphics 19, No 3 (Proc. SIGGRAPH'85), 245-254, 1985.
 * - Pletinckx, D., Quaternion calculus as a basic tool in computer
 *   graphics, The Visual Computer 5, 2-13, 1989.
 */
static void normalize_quat(float q[4])
{
    int i;
    float mag;

    mag = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    for (i = 0; i < 4; i++) q[i] /= mag;
}

/*
 * Build a rotation matrix, given a quaternion rotation.
 *
 */
void build_rotmatrix(float m[4][4], float q[4])
{
    m[0][0] = 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]);
    m[0][1] = 2.0 * (q[0] * q[1] - q[2] * q[3]);
    m[0][2] = 2.0 * (q[2] * q[0] + q[1] * q[3]);
    m[0][3] = 0.0;

    m[1][0] = 2.0 * (q[0] * q[1] + q[2] * q[3]);
    m[1][1]= 1.0 - 2.0 * (q[2] * q[2] + q[0] * q[0]);
    m[1][2] = 2.0 * (q[1] * q[2] - q[0] * q[3]);
    m[1][3] = 0.0;

    m[2][0] = 2.0 * (q[2] * q[0] - q[1] * q[3]);
    m[2][1] = 2.0 * (q[1] * q[2] + q[0] * q[3]);
    m[2][2] = 1.0 - 2.0 * (q[1] * q[1] + q[0] * q[0]);
    m[2][3] = 0.0;

    m[3][0] = 0.0;
    m[3][1] = 0.0;
    m[3][2] = 0.0;
    m[3][3] = 1.0;
}

/* Fortran wrappers */
void trackball_(float *q, float *p1x, float *p1y, float *p2x, float *p2y)
{
   trackball( q, *p1x, *p1y, *p2x, *p2y);
}
  
void add_quats__(float q1[4], float q2[4], float dest[4])
{
        add_quats(q1, q2, dest);
}

void build_rotmatrix__(float m[4][4], float q[4])
{
     /* matrix m is transposed twice. No need to do this here */
     build_rotmatrix(m, q);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//--------------------------------------------->TrackBall Implementation ENds--------------------------------------->
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
GLuint program;

static const GLchar * vertex_shader[] ={"\
varying vec3 normal, lightDir;\
uniform mat4 RotationMatrix;\
void main()\
{          \
  lightDir=normalize(vec3(gl_LightSource[0].position));\
  normal=normalize(gl_NormalMatrix*gl_Normal);\
  gl_Position = gl_ProjectionMatrix * \
  RotationMatrix*gl_ModelViewMatrix*gl_Vertex;\
}"};

static const GLchar * fragment_shader[] ={"\
/* simple toon fragment shader */\
/* www.lighthouse3d.com        */\
\
varying vec3 normal, lightDir;\
\
void main()\
{\
        float intensity;\
        vec3 n;\
        vec4 color;\
\
        n = normalize(normal);\
        intensity = max(dot(lightDir,n),0.0);\
        if (intensity > 0.98)\
                color = vec4(0.8,0.8,0.8,1.0);\
        else if (intensity > 0.5)\
                color = vec4(0.4,0.4,0.8,1.0);\
        else if (intensity > 0.25)\
                color = vec4(0.2,0.2,0.4,1.0);\
        else\
                color = vec4(0.1,0.1,0.1,1.0);\
        gl_FragColor = color;\
}"};

void create_shaders()
{
  GLuint v, f;

  v = glCreateShader(GL_VERTEX_SHADER);
  f = glCreateShader(GL_FRAGMENT_SHADER);
  glShaderSource(v, 1, vertex_shader, NULL);
  glShaderSource(f, 1, fragment_shader, NULL);
  glCompileShader(v);
  GLint compiled;
  glGetShaderiv(v, GL_COMPILE_STATUS, &compiled );
  if ( !compiled ) {
    GLsizei  maxLength, length;
    glGetShaderiv( v, GL_INFO_LOG_LENGTH, &maxLength );
    GLchar* log = (char*)malloc(sizeof(GLchar)*(maxLength+1));
    glGetShaderInfoLog(v,  maxLength, &length, log);
    printf("Vertex Shader compilation failed: %s\n", log);
    free(log);
  }
  glCompileShader(f);
  glGetShaderiv(f, GL_COMPILE_STATUS, &compiled );
  if ( !compiled ) {
    GLsizei  maxLength, length;
    glGetShaderiv( f, GL_INFO_LOG_LENGTH, &maxLength );
    GLchar* log = (char*)malloc(sizeof(GLchar)*(maxLength+1));
    glGetShaderInfoLog(f,  maxLength, &length, log);
    printf("Fragment Shader compilation failed: %s\n", log);
    free(log);
  }
  program = glCreateProgram();
  glAttachShader(program, f);
  glAttachShader(program, v);
  glLinkProgram(program);
  GLint linked;
  glGetProgramiv(program, GL_LINK_STATUS, &linked );
  if ( !linked ) {
    GLsizei len;
    glGetProgramiv(program, GL_INFO_LOG_LENGTH, &len );
    GLchar* log = (char*)malloc(sizeof(GLchar)*(len+1));
    glGetProgramInfoLog(program, len, &len, log );
    printf("Shader linking failed: %s\n", log);
    free(log);
  }
  glUseProgram(program);
}


float lpos[4] = {1, 0.5, 1, 0};
GLfloat m[4][4]; // modelview rotation matrix
float last[4], cur[4]; // rotation tracking quaternions 
int width, height, beginx, beginy;
float p1x, p1y, p2x, p2y;

void display(void) {
  GLuint location = glGetUniformLocation(program, "RotationMatrix");
  build_rotmatrix(m, cur);  
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glLightfv(GL_LIGHT0, GL_POSITION, lpos);
  if( location >= 0 )
    glUniformMatrix4fv(location, 1, GL_FALSE, &m[0][0]);
  //glutSolidTeapot(0.6);
  
  glVertexPointer(3, GL_FLOAT, 0, vertex);
  glEnableClientState(GL_VERTEX_ARRAY);
  glDrawArrays(GL_POINTS, 0, vertices);
  glDisableClientState(GL_VERTEX_ARRAY);

  glutSwapBuffers();
}

void reshape (int w, int h)
{
  double l = 1;
  width=w;  height=h;
  glViewport (0, 0, w, h);
  glMatrixMode (GL_PROJECTION);
  glLoadIdentity();
  glOrtho(-l, l, -l, l, -l, l);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}

void keys(unsigned char key, int x, int y)
{
   if (key == 27 || key == 'q') 
         exit(0);
}

void mouse(int button,int state, int x, int y)   
{
  beginx = x;
  beginy = y;
}

void motion(int x,int y)   
{
  p1x = (2.0*beginx - width)/width;
  p1y = (height - 2.0*beginy)/height;
  p2x = (2.0 * x - width) / width;
  p2y = (height - 2.0 * y) / height;
  trackball(last, p1x, p1y, p2x, p2y);   
  add_quats(last, cur, cur);   
  beginx = x;
  beginy = y;
  glutPostRedisplay();   
}

int main(int argc, char **argv)
{
  ///load  
  int i;
  read_wavefront("rider-body.obj");
  for(i = 0; i < groups; i++) write_wavefront(i);
 
  /////
  
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
  glutInitWindowSize(512, 512);
  glutInitWindowPosition((glutGet(GLUT_SCREEN_WIDTH)-512)/2,
                         (glutGet(GLUT_SCREEN_HEIGHT)-512)/2);
  glutCreateWindow("Use mouse to rotate");
  
  trackball(cur, 0.0, 0.0, 0.0, 0.0);

  glutDisplayFunc(display);
  glutReshapeFunc(reshape);
  glutMouseFunc(mouse);
  glutMotionFunc(motion);
  glutKeyboardFunc(keys);

  glEnable(GL_DEPTH_TEST);
  glClearColor(1.0,1.0,1.0,1.0);
  glewInit();
  if (!glewIsSupported("GL_VERSION_2_0"))
   {
     printf("GLSL not supported\n");
     exit(EXIT_FAILURE);
   }
  create_shaders();
  glutMainLoop();
  return EXIT_SUCCESS;
}