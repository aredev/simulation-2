#include "Particle.h"
#include <GL/glut.h>

Particle::Particle(const Vec2f &ConstructPos, double m) :
        m_ConstructPos(ConstructPos), m_Position(ConstructPos), m_Velocity(Vec2f(0.0, 0.0)) {
    this->mass = m;
}

Particle::~Particle(void) {
}

void Particle::reset() {
    m_Position = m_ConstructPos;
    m_Force = Vec2f(0.0, 0.0);
    m_Velocity = Vec2f(0.0, 0.0);
}

void Particle::draw() {
    const double h = 0.01;
    glColor3f(0.f, 1.f, 1.f);
    glBegin(GL_QUADS);
    glVertex2f(m_Position[0] - h / 2.0, m_Position[1] - h / 2.0);
    glVertex2f(m_Position[0] + h / 2.0, m_Position[1] - h / 2.0);
    glVertex2f(m_Position[0] + h / 2.0, m_Position[1] + h / 2.0);
    glVertex2f(m_Position[0] - h / 2.0, m_Position[1] + h / 2.0);
    glEnd();
}

void Particle::drawForce(){
    glColor3f(0.000, 0.000, 1.000);
    glBegin(GL_LINES);
    glVertex2f(m_Position[0], m_Position[1]);
    glVertex2f(m_Position[0] + m_Force[0] * 10, m_Position[1] + m_Force[1] * 10);
    glEnd();
}

void Particle::drawVelocity(){
    glColor3f(0.000, 1.000, 0.000);
    glBegin(GL_LINES);
    glVertex2f(m_Position[0], m_Position[1]);
    glVertex2f(m_Position[0] + m_Velocity[0], m_Position[1] + m_Velocity[1]);
    glEnd();
}

void Particle::drawDistance(){
    glColor3f(0.000, 1.000, 1.000);
    glBegin(GL_LINES);
    glVertex2f(m_Position[0] + 0.01, m_Position[0] + 0.01);
    glVertex2f(m_Position[0] + 0.01, m_Position[0] + 0.01);
    glEnd();
}