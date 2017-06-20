//
// Created by abdullah on 14/06/2017.
//

#include <GL/gl.h>
#include "Marker.h"

Marker::Marker(const Vec2f &ConstructPos, double m, double density, double velocity) : Particle(ConstructPos, m) {
    this->m_ConstructPos = ConstructPos;
    this->m_Mass = m;
    this->density = density;
    this->velocity = velocity;
}

void Marker::draw() {
    const double h = 0.01;
    glColor3f(density, density, density);
    glBegin(GL_QUADS);
    glVertex2f(m_Position[0] - h / 2.0, m_Position[1] - h / 2.0);
    glVertex2f(m_Position[0] + h / 2.0, m_Position[1] - h / 2.0);
    glVertex2f(m_Position[0] + h / 2.0, m_Position[1] + h / 2.0);
    glVertex2f(m_Position[0] - h / 2.0, m_Position[1] + h / 2.0);
    glEnd();
}
