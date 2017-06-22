#include <GL/gl.h>
#include "WallForce.h"
#include "utility.h"

WallForce::WallForce(Particle* p1, double x, double y) : Force() {
    this->particles.push_back(p1);
    this->p = p1;
    this->x = x;
    this->y = y;
}

WallForce::WallForce(Particle *p1, double x) {
    this->particles.push_back(p1);
    this->p = p1;
    this->x = x;
}

void WallForce::computeForce() {
    if (p->m_Position[0] >= x){
        p->m_Velocity *= -1; //Vec2f(p->m_Velocity[0] + 0.1f, 0.0);
//        p->m_Velocity -= Vec2f(p->m_Velocity[0] + 0.1f, 0.0);
//        p->m_Force -= Vec2f(p->m_Force[0] + 0.1f, 0.0);
        p->m_Force *= -1; //Vec2f(p->m_Force[0] + 0.1f, 0.0);
    }

    if (y != -1){
        if (utility::euclideanDistance(p->m_Position, Vec2f(x,y)) <= epsilon){
            p->m_Velocity *= -1;
            p->m_Force *= -1;
        }
    }
}

void WallForce::draw() {
    glBegin(GL_LINES);
    glColor3f(1, 1, 0);
    glVertex2f(x,-1);
    glVertex2f(x, 1);
    glEnd();
}


