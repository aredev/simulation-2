#include "Particle.h"
#include "SpringForce.h"
#include "RodConstraint.h"
#include "CircularWireConstraint.h"
#include "GravityForce.h"
#include "SlidingWireConstraint.h"
#include "HorizontalForce.h"
#include "WallForce.h"

#include <GL/glut.h>
#include "string.h"
#include "SetUp.h"

/* macros */

/* external definitions (from solver) */
extern void simulation_step(std::vector<Particle *> particle, std::vector<Force *> forces, std::vector<ConstraintForce *> constraints, float dt, int isi);

void createWall(double xPosition);

void createCloth(int N);

/* global variables */

static int N;
static float dt, d;
static int dsim;
static int dump_frames;
static int frame_number;
float proximity_threshold = 0.05f;

/* Mouse dragging variables */

static Vec2f mouseDragOrgPos;
static Vec2f mouseDragNewPos;
static bool isDragging = false;
static bool clothCreated = false;

// static Particle *pList;
static std::vector<Particle *> pVector;
static std::vector<ConstraintForce *> constraintForces;

std::vector<Force *> forceVector;

static int win_id;
static int win_x, win_y;
static int mouse_down[3];
static int mouse_release[3];
static int mouse_shiftclick[3];
static int omx, omy, mx, my;
static int hmx, hmy;
static int clothDimension = 4;
static double particleMass = 1.0f;
static Vec2f center = Vec2f(0.0, 0.0);
static int integrationSchemeIndex = 0;
static std::string integrationSchemeNames[] = {"Forward Euler", "Midpoint", "Runga Kutta 4"};

static SpringForce *delete_this_dummy_spring = NULL;
static RodConstraint *delete_this_dummy_rod = NULL;
static CircularWireConstraint *delete_this_dummy_wire = NULL;


/*
----------------------------------------------------------------------
free/clear/allocate simulation data
----------------------------------------------------------------------
*/

static void free_data(void) {
    pVector.clear();
    constraintForces.clear();
    forceVector.clear();
}

static void clear_data(void) {
    int ii, size = pVector.size();

    for (ii = 0; ii < size; ii++) {
        pVector[ii]->reset();
    }
}

/**
 * Handler for chaning menu options
 * Only works when the simulation is in halt
 * @param item
 */
void onMenuItemChanged(int item){
    free_data();
    clear_data();

    switch (item){
        case 0:
            SetUp::setUpSlidingCloth(pVector, forceVector, constraintForces, 4);
            glutSetWindowTitle("4 by 4 cloth. Right mouse click to slide");
            break;
        case 1:
            SetUp::setUpSlidingClothWall(pVector, forceVector, constraintForces, 4);
            glutSetWindowTitle("4 by 4 cloth. Right mouse click to slide with a wall");
            break;
        case 2:
            SetUp::setUpGravity(pVector, forceVector);
            glutSetWindowTitle("Gravity");
            break;
        case 3:
            SetUp::setUpSpringforce(pVector, forceVector);
            glutSetWindowTitle("Spring m_Force");
            break;
        case 4:
            SetUp::setUpRodConstraint(pVector, forceVector, constraintForces);
            glutSetWindowTitle("Rod constraint with gravity at bottom");
            break;
        case 5:
            SetUp::setUpCircularConstraint(pVector, forceVector, constraintForces);
            glutSetWindowTitle("Circular wire constraint with gravity");
            break;
        case 6:
            SetUp::setUpHorizontalConstraint(pVector, forceVector, constraintForces);
            glutSetWindowTitle("Sliding right over line constraint, with gravity");
            break;
        case 7:
            SetUp::setUpMixedConstraint(pVector, forceVector, constraintForces);
            glutSetWindowTitle("Mixture of several constraints");
            break;
        case 8:
            SetUp::setUpAngularSpring(pVector, forceVector);
            glutSetWindowTitle("Angular constraint");
            break;

    }

}

static void init_system(void) {
    const double dist = 0.2;
    const Vec2f center(0.0, 0.0);
    const Vec2f offset(dist, 0.0);

    double particleMass = 1.0f;

    Particle* centerParticle = new Particle(center, particleMass);
    pVector.push_back(centerParticle);

}

/*
----------------------------------------------------------------------
OpenGL specific drawing routines
----------------------------------------------------------------------
*/
float euclideanDistance(Vec2f a, Vec2f b) {
    return sqrtf(powf(a[0] - b[0], 2) + powf(a[1] - b[1], 2));
}

bool isWithinProximity(Vec2f a, Vec2f b, float threshold) {
    float dist = euclideanDistance(a, b);
    return dist <= threshold;
}

/**
 * Checks whether a given vector is not within a threshold of other particles
 * @param particles
 * @param p
 * @param threshold
 * @return
 */
bool noCloseParticles(std::vector<Particle *> particles, Vec2f v, float threshold) {
    for (auto &particle: particles) {
        float px = particle->m_Position[0];
        float py = particle->m_Position[1];
        if (isWithinProximity(v, particle->m_Position, threshold)) {
            return false;
        }
    }
    return true;
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

/**
 * Transforms x and y coordinates retrieved from a mouse click to the coordinate system used to draw points on the
 * screen.
 * @param x The x value of the mouse
 * @param y  The y value of the mouse
 * @return  Vec2f of transformed coordinates
 */
Vec2f getTransformedCoordinates(int x, int y) {
    float max = 1.0f;
    float min = -1.0f;
    float windowWidth = glutGet(GLUT_WINDOW_WIDTH);
    float windowHeight = glutGet(GLUT_WINDOW_HEIGHT);
    // Normalize the x and y coordinates that indicate the mouse position
    float xPercent = normalize(x, 0, windowWidth);
    float yPercent = normalize(y, 0, windowHeight);
    // Compute values in the new coordinate system
    float destX = xPercent * fabsf(max - min) + min;
    float destY = yPercent * fabsf(max - min) + min;
    destY *= -1;
    return Vec2f(destX, destY);
}

/**
 * Function for handling mouse input.
 * Note that the glut coordinate system is a little bit weird : it corresponds to the coordinate system you are
 * taught in high school.
 * @param button The button
 * @param state The state
 * @param x The x coordinate of the mouse button
 * @param y The y coordinate of the mouse button
 */
void onMouseButton(int button, int state, int x, int y) {
    // TODO make sure we detect end of mouse drag
    int right = GLUT_RIGHT_BUTTON;
    int glutUp = GLUT_UP;
    int glutDown = GLUT_DOWN;
    int glutLeftButton = GLUT_LEFT_BUTTON;
    Vec2f normalizedMouse = getTransformedCoordinates(x, y);
    if (button == GLUT_LEFT_BUTTON && state == GLUT_UP) {
        if (!isDragging) {
            // Left button is released and we were not dragging the mouse
            bool collides = false;
            if (noCloseParticles(pVector, normalizedMouse, proximity_threshold)) {
                pVector.push_back(new Particle(normalizedMouse, 1));
            }
        } else {
            // We finished a mouse drag, so we have to check whether we can place particles on both end points
            mouseDragNewPos = normalizedMouse;
            if (noCloseParticles(pVector, mouseDragOrgPos, proximity_threshold) &&
                noCloseParticles(pVector, mouseDragNewPos, proximity_threshold)) {
                Particle *p1 = new Particle(mouseDragOrgPos, 1);
                Particle *p2 = new Particle(mouseDragNewPos, 1);
                forceVector.push_back(new SpringForce(p1, p2, 0.2f, 0.2f, 0.2f));
                pVector.push_back(p1);
                pVector.push_back(p2);
                isDragging = false;
            }
        }
    }

    if (button == GLUT_RIGHT_BUTTON && state == GLUT_UP){
        //On right mouse click, if there is a cloth we would like to apply a horizontal m_Force on the cloth
        if (clothCreated){
            //If we have a cloth / are in cloth mode, apply the horizontal m_Force on all the particles
            //of the first row, such that we have a sliding effect
            for (int i = 0; i < clothDimension; ++i) {
                Force* horizontalForce = new HorizontalForce(pVector[i], 0.3f);
                forceVector.push_back(horizontalForce);
            }
        }
        //Get the particle which is close and apply the horizontal m_Force on that particle
        Force* horizontalForce = new HorizontalForce(pVector[0], 0.05f);
        forceVector.push_back(horizontalForce);
    }
}

/**
 * https://www.opengl.org/discussion_boards/showthread.php/174097-dragging-using-mouse-movement
 * @param x
 * @param y
 */
void onMouseDrag(int x, int y) {
    Vec2f curPos = getTransformedCoordinates(x, y);
    if (!isDragging) {
        isDragging = true;
        mouseDragOrgPos = curPos;
    } else {
        mouseDragNewPos = curPos;
    }
}

static void pre_display(void) {
    glViewport(0, 0, win_x, win_y);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(-1.0, 1.0, -1.0, 1.0);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    // Register mouse callback function for both clicking and dragging
    glutMouseFunc(onMouseButton);
    glutMotionFunc(onMouseDrag);
}

static void post_display(void) {
    // Write frames if necessary.
    if (dump_frames) {
        const int FRAME_INTERVAL = 4;
        if ((frame_number % FRAME_INTERVAL) == 0) {
            const unsigned int w = glutGet(GLUT_WINDOW_WIDTH);
            const unsigned int h = glutGet(GLUT_WINDOW_HEIGHT);
            unsigned char *buffer = (unsigned char *) malloc(w * h * 4 * sizeof(unsigned char));
            if (!buffer)
                exit(-1);
            // glRasterPos2i(0, 0);
            glReadPixels(0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, buffer);
            static char filename[80];
            sprintf(filename, "snapshots/img%.5i.png", frame_number / FRAME_INTERVAL);
            printf("Dumped %s.\n", filename);
//			saveImageRGBA(filename, buffer, w, h);

            free(buffer);
        }
    }
    frame_number++;

    glutSwapBuffers();
}

static void draw_particles(void) {
    int size = pVector.size();

    for (int ii = 0; ii < size; ii++) {
        pVector[ii]->draw();
        pVector[ii]->drawForce();
        pVector[ii]->drawVelocity();
    }
}

static void draw_constraints(void) {
    for (auto &constraint : constraintForces) {
        constraint->draw();
    }
}


static void draw_forces(void){
    for (auto &force : forceVector){
        force->draw();
    }

}

/*
----------------------------------------------------------------------
relates mouse movements to particle toy construction
----------------------------------------------------------------------
*/

static void get_from_UI() {
    int i, j;
    // int size, flag;
    int hi, hj;
    // float x, y;
    if (!mouse_down[0] && !mouse_down[2] && !mouse_release[0]
        && !mouse_shiftclick[0] && !mouse_shiftclick[2])
        return;

    i = (int) ((mx / (float) win_x) * N);
    j = (int) (((win_y - my) / (float) win_y) * N);

    if (i < 1 || i > N || j < 1 || j > N) return;

    if (mouse_down[0]) {

    }

    if (mouse_down[2]) {
    }

    hi = (int) ((hmx / (float) win_x) * N);
    hj = (int) (((win_y - hmy) / (float) win_y) * N);

    if (mouse_release[0]) {
    }

    omx = mx;
    omy = my;
}

static void remap_GUI() {
    int ii, size = pVector.size();
    for (ii = 0; ii < size; ii++) {
        pVector[ii]->m_Position[0] = pVector[ii]->m_ConstructPos[0];
        pVector[ii]->m_Position[1] = pVector[ii]->m_ConstructPos[1];
    }
}

/*
----------------------------------------------------------------------
GLUT callback routines
----------------------------------------------------------------------
*/

static void key_func(unsigned char key, int x, int y) {
    switch (key) {
        case '1':
            integrationSchemeIndex = 0;
            glutSetWindowTitle(integrationSchemeNames[0].c_str());
            break;
        case '2':
            integrationSchemeIndex = 1;
            glutSetWindowTitle(integrationSchemeNames[1].c_str());
            break;
        case '3':
            integrationSchemeIndex = 2;
            glutSetWindowTitle(integrationSchemeNames[2].c_str());
            break;
        case 'c':
        case 'C':
            clear_data();
            break;

        case 'd':
        case 'D':
            dump_frames = !dump_frames;
            break;

        case 'q':
        case 'Q':
            free_data();
            exit(0);
            break;

        case ' ':
            dsim = !dsim;
            break;
    }
}

static void mouse_func(int button, int state, int x, int y) {
    omx = mx = x;
    omx = my = y;

    if (!mouse_down[0]) {
        hmx = x;
        hmy = y;
    }
    if (mouse_down[button]) mouse_release[button] = state == GLUT_UP;
    if (mouse_down[button]) mouse_shiftclick[button] = glutGetModifiers() == GLUT_ACTIVE_SHIFT;
    mouse_down[button] = state == GLUT_DOWN;
    std::cout << "Clicked" << std::endl;
}

static void motion_func(int x, int y) {
    mx = x;
    my = y;
}

static void reshape_func(int width, int height) {
    glutSetWindow(win_id);
    glutReshapeWindow(width, height);

    win_x = width;
    win_y = height;
}

static void idle_func(void) {
    if (dsim) simulation_step(pVector, forceVector, constraintForces, dt, integrationSchemeIndex);
    else {
        get_from_UI();
        remap_GUI();
    }

    glutSetWindow(win_id);
    glutPostRedisplay();
}

void setUpMenu(){
    glutCreateMenu(onMenuItemChanged);

    glutAddMenuEntry("4 by 4 sliding cloth", 0);
    glutAddMenuEntry("4 by 4 sliding cloth with wall", 1);
    glutAddMenuEntry("Gravity", 2);
    glutAddMenuEntry("Spring Force", 3);
    glutAddMenuEntry("Rod constraint", 4);
    glutAddMenuEntry("Circular wire", 5);
    glutAddMenuEntry("Horizontal wire", 6);
    glutAddMenuEntry("Rod + circular + horizontal", 7);
    glutAddMenuEntry("Angular spring", 8);

    glutAttachMenu(GLUT_MIDDLE_BUTTON);
}

static void display_func(void) {
    pre_display();

    draw_forces();
    draw_constraints();
    draw_particles();

    post_display();
}


/*
----------------------------------------------------------------------
open_glut_window --- open a glut compatible window and set callbacks
----------------------------------------------------------------------
*/

static void open_glut_window(void) {
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);

    glutInitWindowPosition(0, 0);
    glutInitWindowSize(win_x, win_y);
    win_id = glutCreateWindow("Particletoys!");

    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    glutSwapBuffers();
    glClear(GL_COLOR_BUFFER_BIT);
    glutSwapBuffers();

    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_POLYGON_SMOOTH);

    setUpMenu();

    pre_display();

    glutKeyboardFunc(key_func);
    glutMouseFunc(mouse_func);
    glutMotionFunc(motion_func);
    glutReshapeFunc(reshape_func);
    glutIdleFunc(idle_func);
    glutDisplayFunc(display_func);


}


/*
----------------------------------------------------------------------
main --- main routine
----------------------------------------------------------------------
*/

int main2(int argc, char **argv) {
    glutInit(&argc, argv);

    fflush(stdout);
    if (argc == 1) {
        N = 64;
        dt = 0.001f;
        d = 5.f;

        fprintf(stderr, "Using defaults : N=%d dt=%g d=%g\n",
                N, dt, d);
    } else {
        N = atoi(argv[1]);
        dt = atof(argv[2]);
        d = atof(argv[3]);
    }

    printf("\n\nHow to use this application:\n\n");
    printf("\t Toggle construction/simulation display with the spacebar key\n");
    printf("\t Dump frames by pressing the 'd' key\n");
    printf("\t Quit by pressing the 'q' key\n");

    dsim = 0;
    dump_frames = 0;
    frame_number = 0;

    init_system();

    win_x = 512;
    win_y = 512;
    open_glut_window();

    glutMainLoop();

    exit(0);
}

