//
// Created by Tomirio on 19-6-2017.
//

#include <GL/gl.h>
#include <Eigen/Geometry>
#include "RigidBody.h"
#include "../Macros.h"

using namespace Eigen;
using namespace std;


/**
 * Each particle has a constant location in the rigid body space.
 * This is denoted as r_{0i}.
 * The location of the ith particle in world space at time t, denoted ri.t is therefore given by
 * the formula: ri(t)= R(t)*r0i + x(t) where:
 * - R is the rotation matrix
 * - x(t) is the current location of the particle
 */


RigidBody::RigidBody(float *mass, float *v, float *u_prev, float *v_prev, vector<Vec2f> &polyPoints, int N) : Particle(
        Vec2f(0, 0), 1), m_v(v), m_mass(mass), u_prev(u_prev), v_prev(v_prev), N(N), polyEdges(polyPoints) {
    R = Matrix2f::Identity(2, 2);
    calculateCenterOfMass();
    rasterizePolyEdges();
}

RigidBody::RigidBody(float *mass, float *v, float *u_prev, float *v_prev, int N) : Particle(Vec2f(0, 0), 1), m_v(v),
                                                                                   m_mass(mass), u_prev(u_prev),
                                                                                   v_prev(v_prev), N(N) {
    R = Matrix2f::Identity(2, 2);
    calculateCenterOfMass();
    rasterizePolyEdges();
}


/**
 * See https://en.wikipedia.org/wiki/Centroid, section "Centroid of a polygon"
 * https://stackoverflow.com/questions/5271583/center-of-gravity-of-a-polygon
 */
void RigidBody::calculateCenterOfMass() {
    float totalMass = 0;
    Vec2f e1, e2;
    float cx, cy, A, tmp;
    cx = cy = A = 0;
    for (unsigned int i = 0; i < polyEdges.size(); i++) {
        e1 = polyEdges.at(i);
        e2 = polyEdges.at((i + 1) % polyEdges.size());
        tmp = e1[0] * e2[1] - e2[0] * e1[1];
        cx += (e1[0] + e2[0]) * tmp;
        cy += (e1[1] + e2[1]) * tmp;
        A += tmp;
        totalMass += 1;
    }
    m_Mass = totalMass;
    A /= 2.0f;
    m_Position[0] = 1.0f / (6 * A) * cx;
    m_Position[1] = 1.0f / (6 * A) * cy;
}

/**
 * Similarly, we define the total angular
 * momentum L.t/ of a rigid body by the equation L.t/ D I.t/!.t/, where I.t/ is a 33 matrix (technically
 * a rank-two tensor) called the inertia tensor, which we will describe momentarily
 */
void RigidBody::calculateTorque() {
    Vector2f totalTorque, m_Pos, r, force, rabs;;
    m_Pos(0) = m_Position[0];
    m_Pos(1) = m_Position[1];
    int i, j;
    for (auto &polyCell : polyEdgeCells) {
        i = polyCell[0];
        j = polyCell[1];
        r[0] = i - 0.5f * 1 / N;
        r[1] = j - 0.5f * 1 / N;
        // Absolute position
        rabs = r - m_Pos;
        // Apply rotation matrix
        r = R * rabs;
        r += m_Pos;
        // u_prev is de kracht in de x-component van een cel
        // v_prev is de kracht in de y-component van een cel
        force[0] = u_prev[IX(i, j)];
        force[1] = v_prev[IX(i, j)];
        Vector2f tmp = (r - m_Pos);
        tmp(0) *= force(0);
        tmp(1) *= force(1);
        totalTorque += tmp;
    }
    torque(0) = totalTorque[0];
    torque(1) = totalTorque[1];
}

/**
 * See
 * https://physics.stackexchange.com/questions/221078/angular-velocity-of-rigid-body
 * https://math.stackexchange.com/questions/1438191/how-to-find-the-tangential-and-normal-components-of-the-acceleration
 * https://stackoverflow.com/questions/1243614/how-do-i-calculate-the-normal-vector-of-a-line-segment
 * https://stackoverflow.com/questions/7469959/given-2-points-how-do-i-draw-a-line-at-a-right-angle-to-the-line-formed-by-the-t/7470098#7470098
 *
 * this might be it: https://scicomp.stackexchange.com/questions/11353/angular-velocity-by-vector-2d
 */
void RigidBody::calculateAngularVelocity() {
    int i, j;
    float totalOmega = 0;
    for (auto &polyCell : polyEdgeCells) {
        i = polyCell[0];
        j = polyCell[1];
        Vector2f v, r;

        v[1] = u_prev[IX(i, j)];
        v[0] = v_prev[IX(i, j)];

        r[0] = i - 0.5f * 1 / N - m_Position[0];
        r[1] = j - 0.5f * 1 / N - m_Position[1];

        float omega = (r[0] * v[1] - r[1] * v[0]) / (r[0] * r[0] + r[1] * r[1]);
        totalOmega += omega;
    }
    omega = totalOmega / polyEdgeCells.size();
}

/**
 * Compute auxiliary variables
 */
void RigidBody::calculateAuxiliaries() {
    rasterizePolyEdges();
    calculateVelocity();
    calculateTorque();
    calculateMomentumOfIntertia();
    calculateAngularVelocity();
}

/**
 * We should do something similar.
 * https://physics.stackexchange.com/questions/293037/how-to-compute-the-angular-velocity-from-the-angles-of-a-rotation-matrix
 */
void getR() {

}

void RigidBody::calculateMomentumOfIntertia() {
    int i, j;
    float Ic = 0;
    for (auto &polyCell: polyEdgeCells) {
        i = polyCell[0];
        j = polyCell[1];
        float mi = m_mass[IX(i, j)] + 1;
        Vector2f pos;
        pos(0) = i - 0.5f * 1 / N - m_Position[0];
        pos(1) = j - 0.5f * 1 / N - m_Position[1];
        Ic += mi * (pos(0) * pos(0) + pos(1) * pos(1));
    }
    I = Ic;
}

/**
 * Determine the cells that should be selected for all of the edges of the polygon
 */
void RigidBody::rasterizePolyEdges() {
    polyEdgeCells.clear();
    float h = 1.0f / N;
    for (unsigned int i = 0; i < polyEdges.size(); i++) {
        Vec2f p1, p2;
        p1 = polyEdges.at(i);
        p2 = polyEdges.at((i + 1) % polyEdges.size());
        int i1, i2, j1, j2;
        i1 = p1[0] / h + 0.5f;
        j1 = p1[1] / h + 0.5f;
        i2 = p2[0] / h + 0.5f;
        j2 = p2[1] / h + 0.5f;
        BresenhamLineAlgorithm(i1, j1, i2, j2);
    }
}

/**
 * Print all of the grid indices of the points that define the polygon.
 */
void RigidBody::printPolyPointsGridIndices() {
    float h = 1.0f / N;
    for (auto &polyPoint: polyEdges) {
        int i, j;
        i = polyPoint[0] / h + 0.5f;
        j = polyPoint[1] / h + 0.5f;
        printf("[%i, %i]\n", i, j);
    }
}

/**
 * Bresenham's line algorithm (https://rosettacode.org/wiki/Bitmap/Bresenham%27s_line_algorithm)
 * @param x1
 * @param y1
 * @param x2
 * @param y2
 * @param h
 * @param polyCells
 */
void RigidBody::BresenhamLineAlgorithm(float x1, float y1, float x2, float y2) {
    const bool steep = (fabs(y2 - y1) > fabs(x2 - x1));
    if (steep) {
        swap(x1, y1);
        swap(x2, y2);
    }

    if (x1 > x2) {
        swap(x1, x2);
        swap(y1, y2);
    }
    const float dx = x2 - x1;
    const float dy = fabs(y2 - y1);
    float error = dx / 2.0f;
    const int ystep = (y1 < y2) ? 1 : -1;
    int y = (int) y1;
    const int maxX = (int) x2;

    for (int x = (int) x1; x < maxX; x++) {
        Vec2 polyCell;
        if (steep) {
            polyCell = Vec2f(y, x);
        } else {
            polyCell = Vec2f(x, y);
        }
        polyEdgeCells.emplace_back(polyCell);
        error -= dy;
        if (error < 0) {
            y += ystep;
            error += dx;
        }
    }
}

/**
 * Draw the points of the polygon
 */
void RigidBody::drawPolyPoints() {
    const float h = 0.01;
    glColor3f(1.f, 1.f, 1.f);
    for (auto &polyPoint : polyEdges) {
        glBegin(GL_QUADS);
        glVertex2f(polyPoint[0] - h / 2.0f, polyPoint[1] - h / 2.0f);
        glVertex2f(polyPoint[0] + h / 2.0f, polyPoint[1] - h / 2.0f);
        glVertex2f(polyPoint[0] + h / 2.0f, polyPoint[1] + h / 2.0f);
        glVertex2f(polyPoint[0] - h / 2.0f, polyPoint[1] + h / 2.0f);
        glEnd();
    }
}

/**
 * Draw the rigid body
 */
void RigidBody::draw() {
    // Draw all the polygon points and edges
    drawPolyPoints();
    drawPolyEdges();
    // Draw center of mass
    Particle::draw();
}

/**
 * Draw the edges of the polygon
 */
void RigidBody::drawPolyEdges() {
    glColor3f(0.5f, 0.5f, 0.5f);
    Vec2f p1, p2;
    for (unsigned int i = 0; i < polyEdges.size(); i++) {
        p1 = polyEdges.at(i);
        p2 = polyEdges.at((i + 1) % polyEdges.size());
        glBegin(GL_LINES);
        glVertex2f(p1[0], p1[1]);
        glVertex2f(p2[0], p2[1]);
        glEnd();
    }
}

/**
 * v(t) = P(t) / M, where P(t) is the the total linear momentum of the rigid body.
 * P(t) = M * v(t), where M is the mass of the body, and v(t) the velocity of the center particle.
 */
void RigidBody::calculateVelocity() {
    Vec2f P = m_Mass * m_Velocity;
    m_Velocity = P / m_Mass;
}