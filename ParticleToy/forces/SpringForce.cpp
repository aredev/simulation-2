#include "SpringForce.h"
#include <GL/glut.h>

SpringForce::SpringForce(Particle *p1, Particle *p2, double dist, double ks, double kd) :
        m_p1(p1), m_p2(p2), m_dist(dist), m_ks(ks), m_kd(kd) {
    particles.push_back(p1);
    particles.push_back(p2);

}

void SpringForce::computeForce() {
    Vec2f l = this->m_p1->m_Position - this->m_p2->m_Position;
    float l_bars = sqrtf(l * l);

    Vec2f l_dot = this->m_p1->m_Velocity - this->m_p2->m_Velocity;

    Vec2f fp1 = (m_ks * (l_bars - this->m_dist) + m_kd * (l_dot * l) / l_bars) * (l / l_bars);
    m_p1->m_Force -= fp1;
    m_p2->m_Force += fp1;
}

void SpringForce::draw() {
    glBegin(GL_LINES);
    glColor3f(0.6, 0.7, 0.8);
    glVertex2f(m_p1->m_Position[0], m_p1->m_Position[1]);
    glColor3f(0.6, 0.7, 0.8);
    glVertex2f(m_p2->m_Position[0], m_p2->m_Position[1]);
    glEnd();
}

