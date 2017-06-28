//
// Created by Tomirio on 19-6-2017.
//

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
        Vec2f(0, 0), 1), m_v(v), m_mass(mass), u_prev(u_prev), v_prev(v_prev), N(N), polyPoints(polyPoints) {
    R = Matrix2f::Zero(2, 2);
    calculateCenterOfMass();
}


void RigidBody::calculateCenterOfMass() {
    float totalMass = 0;
    Vector2f massPositionSumation = Vector2f::Zero(2);
    int i, j;
    FOR_EACH_CELL
//            if (m_is_polygon_edge[IX(i, j)]) {
//                float mass = m_mass[IX(i, j)];
//                totalMass += mass;
//                Vector2f pos = Vector2f::Zero(2);
//                pos(0) = i - 0.5f * 1 / N;
//                pos(1) = j - 0.5f * 1 / N;
//                massPositionSumation += mass * pos;
//            }
    END_FOR
    m_Mass = totalMass;
    x = totalMass * massPositionSumation;
    m_Position[0] = x(0);
    m_Position[1] = x(1);
}

/**
 * Similarly, we define the total angular
 * momentum L.t/ of a rigid body by the equation L.t/ D I.t/!.t/, where I.t/ is a 33 matrix (technically
 * a rank-two tensor) called the inertia tensor, which we will describe momentarily
 */
void RigidBody::calculateTorque() {
    Vec2f totalTorque;
    int i, j;
//    FOR_EACH_CELL
//    if (m_is_polygon_edge[IX(i, j)]) {
//        float mass = m_mass[IX(i, j)];
//        Vec2f r, force;
//        r[0] = i - 0.5f * 1 / N;
//        r[1] = j - 0.5f * 1 / N;
//        // u_prev is de kracht in de x-component van een cel
//        // v_prev is de kracht in de y-component van een cel
//        force[0] = u_prev[IX(i, j)];
//        force[1] = v_prev[IX(i, j)];
//        totalTorque += (r - m_Position) * force;
//    }
//    END_FOR
//            torque(0) = totalTorque[0];
//    torque(1) = totalTorque[1];
}

/**
 * See https://physics.stackexchange.com/questions/221078/angular-velocity-of-rigid-body
 */
void RigidBody::calculateAngularVelocity() {
//    omega = 1 / I * torque;
}

/**
 * Compute auxiliary variables
 */
void RigidBody::calculateAuxiliaries() {
//    calculateTorque();
//    calculateMomentumOfIntertia();
//    calculateAngularVelocity();
}

/**
 * We should do something similar.
 * https://physics.stackexchange.com/questions/293037/how-to-compute-the-angular-velocity-from-the-angles-of-a-rotation-matrix
 */
void getR() {

}

void RigidBody::calculateMomentumOfIntertia() {
//            int i, j;
//            float Ic = 0;
////    FOR_EACH_CELL
//            float mi = m_mass[IX(i, j)];
//            Vector2f pos;
//            pos(0) = i - 0.5f * 1 / N;
//            pos(1) = j - 0.5f * 1 / N;
//            Ic += mi * (pos(0) * pos(0) + pos(1) * pos(1));
////    END_FOR
//            I = Ic;
}

/**
 * Determine the cells that should be selected for all of the edges of polygon
 */
void RigidBody::rasterizePolyEdges() {
    polyPoints.clear();
    float h = 1 / N;
    for (unsigned int i = 0; i < polyPoints.size(); i++) {
        Vec2f p1, p2;
        if (i == polyPoints.size() - 1) {
            // Final point should be connected to the first point
            p1 = polyPoints.at(i);
            p2 = polyPoints.at(0);
        } else {
            p1 = polyPoints.at(i);
            p2 = polyPoints.at(i + 1);
        }
        int i1, i2, j1, j2;
        i1 = p1[0] / h + 0.5f;
        j1 = p1[1] / h + 0.5f;
        i2 = p2[0] / h + 0.5f;
        j2 = p2[1] / h + 0.5f;
        BresenhamLineAlgorithm(i1, j1, i2, j2, *polyCells);
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
void RigidBody::BresenhamLineAlgorithm(float x1, float y1, float x2, float y2, vector<Vec2f> &polyCells) {
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
        Vec2f polyCell;
        if (steep) {
            polyCell = Vec2f(y, x);
        } else {
            polyCell = Vec2f(x, y);
        }
        polyCells.push_back(polyCell);

        error -= dy;
        if (error < 0) {
            y += ystep;
            error += dx;
        }
    }
}

