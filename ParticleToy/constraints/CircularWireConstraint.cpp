#include "CircularWireConstraint.h"
#include <GL/glut.h>

CircularWireConstraint::CircularWireConstraint(Particle *p, const Vec2f &center, const double radius) :
        m_p(p), m_center(center), m_radius(radius) {
    this->particles.push_back(p);

}

void CircularWireConstraint::draw() {
    glBegin(GL_LINE_LOOP);
    glColor3f(0.0, 1.0, 0.0);
    for (int i = 0; i < 360; i = i + 18) {
        float degInRad = i * M_PI / 180;
        glVertex2f(m_center[0] + cos(degInRad) * m_radius, m_center[1] + sin(degInRad) * m_radius);
    }
    glEnd();
}

double CircularWireConstraint::getC() {
    return pow(this->m_p->m_Position[0] - m_center[0], 2.0) + pow(this->m_p->m_Position[1] - m_center[1], 2.0) - pow(m_radius, 2.0);
}

double CircularWireConstraint::getCdot() {
    return ( 2 * (m_p->m_Position - m_center)) * (2 * m_p->m_Velocity);
}

VectorXd CircularWireConstraint::getq() {
    VectorXd MatrixQ(2);
    MatrixQ(0) = m_p->m_Position[0];
    MatrixQ(1) = m_p->m_Position[1];
    return MatrixQ;
}

MatrixXd CircularWireConstraint::getJ() {
    MatrixXd MatrixJ(1, 2);
    MatrixJ(0, 0) = 2 * (m_p->m_Position[0] - m_center[0]);
    MatrixJ(0, 1) = 2 * (m_p->m_Position[1] - m_center[1]);

    return MatrixJ;
}

MatrixXd CircularWireConstraint::getW() {
    MatrixXd MatrixW(2, 2);
    MatrixW(0, 0) = 1 / (m_p->m_Mass);
    MatrixW(1, 1) = 1 / (m_p->m_Mass);
    return MatrixW;
}

MatrixXd CircularWireConstraint::getJDot() {
    MatrixXd MatrixJ(1, 2);
    MatrixJ(0, 0) = 2 * (m_p->m_Velocity[0]);
    MatrixJ(0, 1) = 2 * (m_p->m_Velocity[1]);

    return MatrixJ;
}

VectorXd CircularWireConstraint::getqDot() {
    VectorXd MatrixQ(2);
    MatrixQ(0) = m_p->m_Velocity[0];
    MatrixQ(1) = m_p->m_Velocity[1];
    return MatrixQ;
}

VectorXd CircularWireConstraint::getQ() {
    VectorXd MatrixQ(2);
    MatrixQ(0) = m_p->m_Force[0];
    MatrixQ(1) = m_p->m_Force[1];
    return MatrixQ;
}
