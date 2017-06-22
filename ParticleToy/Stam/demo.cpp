/*
  ======================================================================
   demo.c --- protoype to show off the simple solver
  ----------------------------------------------------------------------
   Author : Jos Stam (jstam@aw.sgi.com)
   Creation Date : Jan 9 2003

   Description:

	This code is a simple prototype that demonstrates how to use the
	code provided in my GDC2003 paper entitles "Real-Time Fluid Dynamics
	for Games". This code uses OpenGL and GLUT for graphics and interface

  =======================================================================
*/

#include <stdlib.h>
#include <stdio.h>
#include <GL/glut.h>
#include <../include/gfx/vec2.h>
#include "../Particle.h"
#include "Cell.h"
#include "../Force.h"
#include "../ConstraintForce.h"
#include "../GravityForce.h"
#include "Marker.h"
#include "../SolidFluidForce.h"
#include "../SpringForce.h"
#include "../BoundaryForce.h"

/* macros */

#define IX(i,j) ((i)+(N+2)*(j))

/* external definitions (from solver.c) */

extern void dens_step ( int N, float * x, float * x0, float * u, float * v, float diff, float dt, bool* b );
extern void vel_step ( int N, float * u, float * v, float * u0, float * v0, float visc, float dt, bool* b );
extern void simulation_step(std::vector<Particle *> particles,
                            std::vector<Force *> forces,
                            std::vector<ConstraintForce *> constraints,
                            float dtx,
                            int integrationSchemeIndex);

static void draw_particles();

static void transform_to_markers();

static void draw_solid() ;

static void draw_forces(void);

static void find_particle_at(int x, int y);

void draw_cloth();
/**
 * Draws 4 particles, interconnected using four springs.
 */
void draw_simple_solid_object() ;

/* global variables */

static int N;
static float dt, diff, visc;
static float force, source;
static int dvel;

static float * u, * v, * u_prev, * v_prev;
static float * dens, * dens_prev;
static bool * b; //boundary
static bool withBool = false;
static bool particleDraggable = false;
static std::vector<Vec2f> boundaries;

static int win_id;
static int win_x, win_y;
static int mouse_down[3];
static int omx, omy, mx, my;

static Vec2f F_marks[64*64] = { Vec2f(0.0f, 0.0f) }; //2D array to hold the force on the marker per (i, j)

std::vector<Particle *> pVector;
std::vector<Marker *> markers;
std::vector<Particle *> solidParticles;
std::vector<Force *> forceVector;
std::vector<ConstraintForce *> constraintForces;



/*
  ----------------------------------------------------------------------
   free/clear/allocate simulation data
  ----------------------------------------------------------------------
*/


static void free_data ( void )
{
    if ( u ) free ( u );
    if ( v ) free ( v );
    if ( u_prev ) free ( u_prev );
    if ( v_prev ) free ( v_prev );
    if ( dens ) free ( dens );
    if ( dens_prev ) free ( dens_prev );
    if ( b ) free ( b );
}

static void clear_data ( void )
{
    int i, size=(N+2)*(N+2);

    for ( i=0 ; i<size ; i++ ) {
        u[i] = v[i] = u_prev[i] = v_prev[i] = dens[i] = dens_prev[i] = 0.0f;
    }

    for (auto& particle: solidParticles) {
        particle->reset();
        forceVector.clear();
    }

    draw_simple_solid_object();
}

static int allocate_data ( void )
{
    int size = (N+2)*(N+2);

    u			= (float *) malloc ( size*sizeof(float) );
    v			= (float *) malloc ( size*sizeof(float) );
    u_prev		= (float *) malloc ( size*sizeof(float) );
    v_prev		= (float *) malloc ( size*sizeof(float) );
    dens		= (float *) malloc ( size*sizeof(float) );
    dens_prev	= (float *) malloc ( size*sizeof(float) );
    b	        = (bool *) malloc ( size*sizeof(bool) );

    if ( !u || !v || !u_prev || !v_prev || !dens || !dens_prev || !b) {
        fprintf ( stderr, "cannot allocate data\n" );
        return ( 0 );
    }

    return ( 1 );
}


/*
  ----------------------------------------------------------------------
   OpenGL specific drawing routines
  ----------------------------------------------------------------------
*/

static void pre_display ( void )
{
    glViewport ( 0, 0, win_x, win_y );
    glMatrixMode ( GL_PROJECTION );
    glLoadIdentity ();
    gluOrtho2D ( 0.0, 1.0, 0.0, 1.0 );
    glClearColor ( 0.0f, 0.0f, 0.0f, 1.0f );
    glClear ( GL_COLOR_BUFFER_BIT );
}

static void post_display ( void )
{
    glutSwapBuffers ();
}

static void draw_velocity ( void )
{
    int i, j;
    float x, y, h;

    h = 1.0f/N;

    glColor3f ( 1.0f, 1.0f, 1.0f );
    glLineWidth ( 1.0f );

    glBegin ( GL_LINES );

    for ( i=1 ; i<=N ; i++ ) {
        x = (i-0.5f)*h;
        for ( j=1 ; j<=N ; j++ ) {
            y = (j-0.5f)*h;

            glVertex2f ( x, y );
            glVertex2f ( x+u[IX(i,j)], y+v[IX(i,j)] );
        }
    }

    glEnd ();
}

static void draw_boundary ( void ){
    glColor3f ( 0.5f, 0.5, 0.5f );

    glBegin ( GL_QUADS );

    float x, y, h, d00, d01, d10, d11;
    h = 1.0f/N;

    for (int i = 0; i < N; ++i) {
        x = (i-0.5f)*h;
        for (int j = 0; j < N; ++j) {
            y = (j-0.5f)*h;
            if (b[IX(i,j)]){

                d00 = b[IX(i,j)];
                d01 = b[IX(i,j+1)];
                d10 = b[IX(i+1,j)];
                d11 = b[IX(i+1,j+1)];

                glVertex2f ( x, y );
                glVertex2f ( x+h, y );
                glVertex2f ( x+h, y+h );
                glVertex2f ( x, y+h );
            }
        }
    }

    glEnd();
}

static void draw_density ( void )
{
    int i, j;
    float x, y, h, d00, d01, d10, d11;

    h = 1.0f/N;

    glBegin ( GL_QUADS );

    for ( i=0 ; i<=N ; i++ ) {
        x = (i-0.5f)*h;
        for ( j=0 ; j<=N ; j++ ) {
            y = (j-0.5f)*h;

            d00 = dens[IX(i,j)];
            d01 = dens[IX(i,j+1)];
            d10 = dens[IX(i+1,j)];
            d11 = dens[IX(i+1,j+1)];

            glColor3f ( d00, d00, d00 ); glVertex2f ( x, y );
            glColor3f ( d10, d10, d10 ); glVertex2f ( x+h, y );
            glColor3f ( d11, d11, d11 ); glVertex2f ( x+h, y+h );
            glColor3f ( d01, d01, d01 ); glVertex2f ( x, y+h );
        }
    }

    glEnd ();
}

/*
  ----------------------------------------------------------------------
   relates mouse movements to forces sources
  ----------------------------------------------------------------------
*/

float euclideanDistance(Vec2f a, Vec2f b) {
    return sqrtf(powf(a[0] - b[0], 2) + powf(a[1] - b[1], 2));
}

bool isWithinProximity(Vec2f a, Vec2f b, float threshold) {
    float dist = euclideanDistance(a, b);
    return dist <= threshold;
}

static void get_from_UI ( float * d, float * u, float * v )
{
    int i, j, size = (N+2)*(N+2);

    for ( i=0 ; i<size ; i++ ) {
        u[i] = v[i] = d[i] = 0.0f;
    }

    if ( !mouse_down[0] && !mouse_down[2] ) return;

    i = (int)((       mx /(float)win_x)*N+1);
    j = (int)(((win_y-my)/(float)win_y)*N+1);

    if ( i<1 || i>N || j<1 || j>N ) return;

    if ( mouse_down[0] && !particleDraggable ) {
        u[IX(i,j)] = force * (mx-omx);
        v[IX(i,j)] = force * (omy-my);
    }

    if ( mouse_down[0] && withBool) {
        b[IX(i,j)] = true;
    }

    if ( mouse_down[2] ) {
        d[IX(i,j)] = source;
    }

    omx = mx;
    omy = my;

    return;
}

/*
  ----------------------------------------------------------------------
   GLUT callback routines
  ----------------------------------------------------------------------
*/

static void key_func ( unsigned char key, int x, int y )
{
    switch ( key )
    {
        case 'b':
        case 'B':
            withBool = !withBool;
            printf("Boundary mode %d\n", withBool);
            break;
        case 'c':
        case 'C':
            clear_data ();
            break;
        case 'd':
        case 'D':
            particleDraggable = !particleDraggable;
            printf("Particle draggable %d \n", particleDraggable);
            break;
        case 'q':
        case 'Q':
            free_data ();
            exit ( 0 );
            break;

        case 'v':
        case 'V':
            dvel = !dvel;
            break;
    }
}

/**
 * See https://gamedev.stackexchange.com/questions/32555/how-do-i-convert-between-two-different-2d-coordinate-systems
 * @param value The value to normalize
 * @param min The min value
 * @param max The max value
 * @return
 */
float normalize(float value, float min, float max) {
    return fabsf((value - min) / (max - min));
}

Vec2f getTransformedCoordinates(int x, int y) {
    float max = 1.0f;
    float min = 0.0f;
    float windowWidth = glutGet(GLUT_WINDOW_WIDTH);
    float windowHeight = glutGet(GLUT_WINDOW_HEIGHT);
    // Normalize the x and y coordinates that indicate the mouse position
    float xPercent = normalize(x, 0, windowWidth);
    float yPercent = normalize(y, 0, windowHeight);
    // Compute values in the new coordinate system
    float destX = xPercent * fabsf(max - min) + min;
    float destY = yPercent * fabsf(max - min) + min;
    return Vec2f(destX, destY);
}

static Particle* find_particle_at(float x, float y) {
    for(auto &particle : solidParticles){
        if (isWithinProximity(Vec2f(x,y), particle->m_Position, 0.15f)){
            return particle;
        }
    }
}

static void mouse_func (int button, int state, int x, int y )
{
    omx = mx = x;
    omx = my = y;

    mouse_down[button] = state == GLUT_DOWN;
}

static void motion_func ( int x, int y )
{
    mx = x;
    my = y;

    if (particleDraggable) {
        Vec2f normalizedMouse = getTransformedCoordinates(x, y);
        Particle* p = find_particle_at(normalizedMouse[0], normalizedMouse[1]);
        p->m_Position = normalizedMouse;
        p->setColour(0.5f);
    }

    if (withBool){
        boundaries.push_back(Vec2f(x,y));
    }

}

static void reshape_func ( int width, int height )
{
    glutSetWindow ( win_id );
    glutReshapeWindow ( width, height );

    win_x = width;
    win_y = height;
}

static void idle_func ( void )
{
    get_from_UI ( dens_prev, u_prev, v_prev );
//    transform_to_markers();
    simulation_step(solidParticles, forceVector, constraintForces, dt, 0);
    //u_prev, v_prev are the added sources.
    vel_step ( N, u, v, u_prev, v_prev, visc, dt, b );
    dens_step ( N, dens, dens_prev, u, v, diff, dt, b );

    glutSetWindow ( win_id );
    glutPostRedisplay ();
}

static void transform_to_markers() {
    markers.clear();
    int i, j;
    float x, y, h, d00, d01, d10, d11;

    h = 1.0f/N;

    for ( i=0 ; i<=N ; i++ ) {
        x = (i-0.5f)*h;
        for ( j=0 ; j<=N ; j++ ) {
            y = (j-0.5f)*h;

            d00 = dens[IX(i,j)];
            if (d00 > 0){
                markers.push_back(new Marker(Vec2f(x, y), 1.0f, d00, Vec2f(u[IX(i,j)], v[IX(i,j)]) ));
            }
        }
    }

}

static void display_func ( void )
{
    pre_display ();

    if ( dvel ) draw_velocity ();
    else		draw_density ();

    draw_particles();
    draw_boundary();
    draw_solid();
    draw_forces();

    post_display ();
}

static void draw_particles() {
    for (auto &partice : markers){
        partice->draw();
    }
}

static void draw_forces(void){
    for (auto &force : forceVector){
        force->draw();
    }
}

static void draw_solid() {
    for (auto &solid : solidParticles){
        solid->draw();
        solid->drawForce();
        solid->drawVelocity();
    }
}

/**
 * Draws 4 particles, interconnected using four springs.
 */
void draw_simple_solid_object() {
    float x, y, h;

    h = 1.0f/N;
    x = (N/2-0.5f)*h;
    y = (N/2-0.5f)*h;

    Vec2f center = Vec2f(x, y);
    Vec2f offset = Vec2f(x/2, y/2);
    Particle* p = new Particle(center, 1.0f);
    Particle* pright = new Particle(center+Vec2f(0.2f, 0.2f), 1.0f);
    Particle* pbottom = new Particle(center-Vec2f(0.3f, 0.3f), 1.0f);

//    solidParticles.push_back(pbottom);
    solidParticles.push_back(p);
//    solidParticles.push_back(pright);

//    forceVector.push_back(new SpringForce(pbottom, p, 0.2, 0.1, 0.1));
//    forceVector.push_back(new SpringForce(p, pright, 0.2, 0.1, 0.1));

    forceVector.push_back(new SolidFluidForce(solidParticles, u, v, u_prev, v_prev, dens));

    forceVector.push_back(new BoundaryForce(solidParticles, boundaries));

//    forceVector.push_back(new GravityForce(solidParticles));
}



/*
  ----------------------------------------------------------------------
   open_glut_window --- open a glut compatible window and set callbacks
  ----------------------------------------------------------------------
*/

static void open_glut_window ( void )
{
    glutInitDisplayMode ( GLUT_RGBA | GLUT_DOUBLE );

    glutInitWindowPosition ( 0, 0 );
    glutInitWindowSize ( win_x, win_y );
    win_id = glutCreateWindow ( "Alias | wavefront" );

    glClearColor ( 0.0f, 0.0f, 0.0f, 1.0f );
    glClear ( GL_COLOR_BUFFER_BIT );
    glutSwapBuffers ();
    glClear ( GL_COLOR_BUFFER_BIT );
    glutSwapBuffers ();

    pre_display ();

    glutKeyboardFunc ( key_func );
    glutMouseFunc ( mouse_func );
    glutMotionFunc ( motion_func );
    glutReshapeFunc ( reshape_func );
    glutIdleFunc ( idle_func );
    glutDisplayFunc ( display_func );

}


/*
  ----------------------------------------------------------------------
   main --- main routine
  ----------------------------------------------------------------------
*/

int main ( int argc, char ** argv )
{
    glutInit ( &argc, argv );

    if ( argc != 1 && argc != 6 ) {
        fprintf ( stderr, "usage : %s N dt diff visc force source\n", argv[0] );
        fprintf ( stderr, "where:\n" );\
		fprintf ( stderr, "\t N      : grid resolution\n" );
        fprintf ( stderr, "\t dt     : time step\n" );
        fprintf ( stderr, "\t diff   : diffusion rate of the density\n" );
        fprintf ( stderr, "\t visc   : viscosity of the fluid\n" );
        fprintf ( stderr, "\t force  : scales the mouse movement that generate a force\n" );
        fprintf ( stderr, "\t source : amount of density that will be deposited\n" );
        exit ( 1 );
    }

    if ( argc == 1 ) {
        N = 64;
        dt = 0.1f;
        diff = 0.0f;
        visc = 0.0f;
        force = 5.0f;
        source = 100.0f;
        fprintf ( stderr, "Using defaults : N=%d dt=%g diff=%g visc=%g force = %g source=%g\n",
                  N, dt, diff, visc, force, source );
    } else {
        N = atoi(argv[1]);
        dt = atof(argv[2]);
        diff = atof(argv[3]);
        visc = atof(argv[4]);
        force = atof(argv[5]);
        source = atof(argv[6]);
    }

    printf ( "\n\nHow to use this demo:\n\n" );
    printf ( "\t Add densities with the right mouse button\n" );
    printf ( "\t Toggle draw boundary mode by pressing 'b' key\n");
    printf ( "\t Add velocities with the left mouse button and dragging the mouse\n" );
    printf ( "\t Toggle density/velocity display with the 'v' key\n" );
    printf ( "\t Clear the simulation by pressing the 'c' key\n" );
    printf ( "\t Quit by pressing the 'q' key\n" );

    dvel = 0;

    if ( !allocate_data () ) exit ( 1 );
    clear_data ();

    //Init system
    draw_simple_solid_object();

//    draw_cloth();

    win_x = 512;
    win_y = 512;
    open_glut_window ();

    glutMainLoop ();

    exit ( 0 );
}

void draw_cloth() {
    float x, y, h;
    h = 1.0f/N;
    x = (N/2-0.5f)*h;
    y = (N/2-0.5f)*h;

    Vec2f center = Vec2f(x, y);
    double spacing = 0.05;

    int SIZE = 3;

    for (int i = 0; i < SIZE; ++i) {
        for (int j = 0; j < SIZE; ++j) {
            Particle *p = new Particle(Vec2f(center[0] + j * spacing, center[1] - i * spacing), 1.0f);
            solidParticles.push_back(p);

            if (j > 0) {
                forceVector.push_back(new SpringForce(solidParticles[SIZE * i + j], solidParticles[SIZE * i + j - 1], spacing, 1, 1));
            }

            if (i > 0) {
                forceVector.push_back(new SpringForce(solidParticles[SIZE * i + j], solidParticles[SIZE * i + j - SIZE], spacing, 1, 1));
            }
        }
    }

    forceVector.push_back(new SolidFluidForce(solidParticles, u, v, u_prev, v_prev, dens));
}