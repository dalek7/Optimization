#ifndef __DDGLPRIMITIVES_H__
#define __DDGLPRIMITIVES_H__

#include <GL/glut.h>

namespace DD {
// -lGL -lGLU -lglut

#ifndef PI
#define PI              3.14159265358979323   /* 180 deg */
#endif

#ifndef DEG
#define DEG(a)          (180.*a/PI )
#endif

#ifndef RAD
#define RAD(a)          (PI*a/180.)
#endif

// Arrow based on QGLViewer
// See https://github.com/GillesDebunne/libQGLViewer/blob/master/QGLViewer/qglviewer.cpp
// Uses current color and does not modify the OpenGL state.
static void drawArrow(float length, float radius, int nbSubdivisions=12)
{
	static GLUquadric* quadric = gluNewQuadric();

	if (radius < 0.0)
		radius = 0.05 * length;

	const float head = 2.5*(radius / length) + 0.1;
	const float coneRadiusCoef = 4.0 - 5.0 * head;

	gluCylinder(quadric, radius, radius, length * (1.0 - head/coneRadiusCoef), nbSubdivisions, 1);
	glTranslated(0.0, 0.0, length * (1.0 - head));
	gluCylinder(quadric, coneRadiusCoef * radius, 0.0, head * length, nbSubdivisions, 1);
	glTranslated(0.0, 0.0, -length * (1.0 - head));
}

/*! Draws a 3D arrow between the 3D point \p from and the 3D point \p to, both defined in the
current ModelView coordinates system.
See drawArrow(float length, float radius, int nbSubdivisions) for details. */
static void drawArrow(const float* from, const float* to, float radius, int nbSubdivisions=12)
{

	glPushMatrix();
	glTranslated(from[0], from[1], from[2]);

	/* do it later !
	const Vec dir = to-from;
	glMultMatrixd(Quaternion(Vec(0,0,1), dir).matrix());
	drawArrow(dir.norm(), radius, nbSubdivisions);

	*/
	glPopMatrix();
}


// based on ORB SLAM
static void drawCamera(float g)
{
	glDisable (GL_LIGHTING);

	const float w = g;
    const float h = w*0.75;
    const float z = w*0.6;

	glPushMatrix();

	glLineWidth(1);

	//glColor3f(0.0f,0.0f,1.0f);
	glPointSize(5.0);
    glBegin (GL_POINTS);
    {
        glVertex3d(0,0,0);

    }
    glEnd();
    glPointSize(1.0);

    glBegin(GL_LINES);
		glVertex3f(0,0,0);
		glVertex3f(w,h,z);
		glVertex3f(0,0,0);
		glVertex3f(w,-h,z);
		glVertex3f(0,0,0);
		glVertex3f(-w,-h,z);
		glVertex3f(0,0,0);
		glVertex3f(-w,h,z);

		glVertex3f(w,h,z);
		glVertex3f(w,-h,z);

		glVertex3f(-w,h,z);
		glVertex3f(-w,-h,z);

		glVertex3f(-w,h,z);
		glVertex3f(w,h,z);

		glVertex3f(-w,-h,z);
		glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();


	glEnable (GL_LIGHTING);



}


static void glCross3f(GLfloat x, GLfloat y,GLfloat z, GLfloat d)
{
	glPushMatrix();

	glBegin(GL_LINE_STRIP);
		glVertex3f(x - d/2 , y, z);
		glVertex3f(x + d/2 , y, z);
	glEnd();

	glBegin(GL_LINE_STRIP);
		glVertex3f(x , y - d/2, z);
		glVertex3f(x , y + d/2, z);
	glEnd();


	glPopMatrix();
}

static void glCircle3f(GLfloat x, GLfloat y,GLfloat z, GLfloat r)
{
	float step = 360.0 / (2.0*PI*r) / 16;
	glPushMatrix();
	glTranslatef(x, y, z);
	glBegin(GL_LINE_STRIP);
	//glVertex2f(0, 0);
	for(float i=0; i<360; i+=step)
	{
		glVertex3f((cos(RAD(i))*r), (sin(RAD(i))*r), 0);
	}
	glEnd();
	glPopMatrix();
}



static void glCircle(GLfloat x, GLfloat y, GLfloat r)
{
	float step = 360.0 / (2.0*PI*r) / 10;
	glPushMatrix();
	glTranslatef(x, y, 0);
	glBegin(GL_LINE_STRIP);
	//glVertex2f(0, 0);
	for(float i=0; i<360; i+=step)
	{
		glVertex2f((cos(RAD(i))*r), (sin(RAD(i))*r));
	}
	glEnd();
	glPopMatrix();
}


//http://ozark.hendrix.edu/~burch/cs/490/sched/feb8/
//drawSphere(1.0, 10, 10);
inline void drawSphere(double r, int lats, int longs)
{

	int i, j;
	for(i = 0; i <= lats; i++) {
	   double lat0 = PI * (-0.5 + (double) (i - 1) / lats);
	   double z0  = sin(lat0);
	   double zr0 =  cos(lat0);

	   double lat1 = PI * (-0.5 + (double) i / lats);
	   double z1 = sin(lat1);
	   double zr1 = cos(lat1);

	   glBegin(GL_QUAD_STRIP);
	   for(j = 0; j <= longs; j++) {
		   double lng = 2 * PI * (double) (j - 1) / longs;
		   double x = cos(lng);
		   double y = sin(lng);

		   glNormal3f(r* x * zr0, r* y * zr0, r* z0);
		   glVertex3f(r* x * zr0, r* y * zr0, r* z0);
		   glNormal3f(r* x * zr1, r* y * zr1, r* z1);
		   glVertex3f(r* x * zr1, r* y * zr1, r* z1);
	   }
	   glEnd();
	}
}


/*! Draws a grid in the XY plane, centered on (0,0,0) (defined in the current coordinate system).
\p size (OpenGL units) and \p nbSubdivisions define its geometry. Set the \c GL_MODELVIEW matrix to
place and orientate the grid in 3D space (see the drawAxis() documentation).
The OpenGL state is not modified by this method. */
static void drawGrid(float size, int nbSubdivisions)
{
	GLboolean lighting;
	glGetBooleanv(GL_LIGHTING, &lighting);

	glDisable(GL_LIGHTING);

	glBegin(GL_LINES);
	for (int i=0; i<=nbSubdivisions; ++i)
	{
		const float pos = size*(2.0*i/nbSubdivisions-1.0);
		glVertex2d(pos, -size);
		glVertex2d(pos, +size);
		glVertex2d(-size, pos);
		glVertex2d( size, pos);
	}
	glEnd();

	if (lighting)
		glEnable(GL_LIGHTING);
}




static void DrawGrid(float size=10, float step=1, bool xz=true)
{
	// disable lighting
	glDisable(GL_LIGHTING);

	glBegin(GL_LINES);

	glColor3f(0.3f, 0.3f, 0.3f);
	for(float i=step; i <= size; i+= step)
	{


		if(xz)
		{
			glVertex3f(-size, 0,  i);   // lines parallel to X-axis
			glVertex3f( size, 0,  i);
			glVertex3f(-size, 0, -i);   // lines parallel to X-axis
			glVertex3f( size, 0, -i);

			glVertex3f( i, 0, -size);   // lines parallel to Z-axis
			glVertex3f( i, 0,  size);
			glVertex3f(-i, 0, -size);   // lines parallel to Z-axis
			glVertex3f(-i, 0,  size);
		}
		else
		{
			glVertex3f(-size,  i, 0);   // lines parallel to X-axis
			glVertex3f( size,  i, 0);
			glVertex3f(-size, -i, 0);   // lines parallel to X-axis
			glVertex3f( size, -i, 0);


			glVertex3f( i,  -size,0);   // lines parallel to Z-axis
			glVertex3f( i,   size,0);
			glVertex3f(-i,  -size,0);   // lines parallel to Z-axis
			glVertex3f(-i,   size,0);


		}
	}

	// x-axis
	glColor3f(0.5f, 0, 0);
	glVertex3f(-size, 0, 0);
	glVertex3f( size, 0, 0);

	if(xz)
	{
		// z-axis
		glColor3f(0,0,0.5f);
		glVertex3f(0, 0, -size);
		glVertex3f(0, 0,  size);

	}
	else
	{
		// z-axis
		glColor3f(0,0.5f,0);
		glVertex3f(0, -size,0);
		glVertex3f(0,  size,0);
	}


	glEnd();

	// enable lighting back
	glColor3f(1,1,1);
	glEnable(GL_LIGHTING);
}

}	// namespace
#endif
