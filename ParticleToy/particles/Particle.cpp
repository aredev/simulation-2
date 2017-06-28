#include "Particle.h"
#include <GL/glut.h>

Particle::Particle(const Vec2f &ConstructPos, float m) :
        m_ConstructPos(ConstructPos), m_Position(ConstructPos), m_Velocity(Vec2f(0.0, 0.0)) {
    this->m_Mass = m;
}

Particle::~Particle(void) {
}

void Particle::reset() {
    m_Position = m_ConstructPos;
    m_Velocity = Vec2f(0.0, 0.0);
}

void Particle::draw() {
    const float h = 0.001;
    glColor3f(1.f, 1.f, 1.f);
    glBegin(GL_QUADS);
    glVertex2f(m_Position[0] - h / 2.0f, m_Position[1] - h / 2.0f);
    glVertex2f(m_Position[0] + h / 2.0f, m_Position[1] - h / 2.0f);
    glVertex2f(m_Position[0] + h / 2.0f, m_Position[1] + h / 2.0f);
    glVertex2f(m_Position[0] - h / 2.0f, m_Position[1] + h / 2.0f);
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