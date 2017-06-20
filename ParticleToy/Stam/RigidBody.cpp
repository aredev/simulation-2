//
// Created by Tomirio on 19-6-2017.
//

#include "RigidBody.h"
#include "Macros.h"
#include <Eigen/LU>

using namespace Eigen;


/**
 * Each particle has a constant location in the rigid body space.
 * This is denoted as r_{0i}.
 * The location of the ith particle in world space at time t, denoted ri.t is therefore given by
 * the formula: ri(t)= R(t)*r0i + x(t) where:
 * - R is the rotation matrix
 * - x(t) is the current location of the particle
 */


RigidBody::RigidBody(float *is_polygon_edge, float *mass, float *v, int N) : Particle(Vec2f(0, 0), 1),
                                                                             m_is_polygon_edge(is_polygon_edge),
                                                                             m_v(v),
                                                                             N(N) {
    R = Matrix2f::Zero(2, 2);
    calculateCenterOfMass();
    calculateIbody();
}

void RigidBody::calculateCenterOfMass() {
    float totalMass = 0;
    Vector2f massPositionSumation = Vector2f::Zero(2);
    int i, j;
    FOR_EACH_CELL
            if (m_is_polygon_edge[IX(i, j)]) {
                float mass = m_mass[IX(i, j)];
                totalMass += mass;
                Vector2f pos = Vector2f::Zero(2);
                pos(0) = i - 0.5f * 1 / N;
                pos(1) = j - 0.5f * 1 / N;
                massPositionSumation += mass * pos;
            }
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
    MatrixXd totalTorque;
    int i, j;
    FOR_EACH_CELL
            if (m_is_polygon_edge[IX(i, j)]) {
                float mass = m_mass[IX(i, j)];
                MatrixXd pos = MatrixXd::Zero(2, 1);
                pos(0) = i - 0.5f * 1 / N;
                pos(1) = j - 0.5f * 1 / N;
                float interia2D = mass * (pos(0) * pos(0) + pos(1) * pos(1));

            }
    END_FOR
//    for (float individualMass : m_bodyMasses) {
//        MatrixXd r = m_R * m_constantBodyLocations[i] + m_currentBodyLocations[i];
//        MatrixXd tau = (r - m_centerOfMass); // * m_F[i] TODO multiply with force applied on the specific particle
//    }
}


void RigidBody::calculateLinearMomentum() {
    int i, j;
    float linearMomentum = 0;
    FOR_EACH_CELL
            if (m_is_polygon_edge[IX(i, j)]) {
                float mass = m_mass[IX(i, j)];
                float veloc = m_v[IX(i, j)];
                linearMomentum += mass * veloc;
            }
    END_FOR
    m_Force = linearMomentum;
}

void RigidBody::calculateIinverse() {
    Iinv = R * Ibodyinv * R.transpose();
}

void RigidBody::calculateV() {
    m_Velocity = m_Force / m_Mass;
}

void RigidBody::calculateOmega() {
    omega = Iinv * L;
}

void RigidBody::calculateIbody() {
    Matrix2f I, tmp;
    int i, j;
    Ibody = Matrix2f::Zero(2, 2);
    I = Matrix2f::Identity();
    Vector2f centerOfMassPos(2);
    FOR_EACH_CELL
            Vector2f pos = Vector2f::Zero(2);
            pos(0) = i - 0.5f * 1 / N;
            pos(1) = j - 0.5f * 1 / N;
            Vector2f r0 = pos - centerOfMassPos;
            float mass = m_mass[IX(i, j)];
            tmp = ((r0.transpose() * r0) * I - r0 * r0.transpose());
            tmp *= mass;
            Ibody += tmp;
    END_FOR
    Ibodyinv = Ibody.inverse();
}

/**
 * Compute auxiliary variables
 */
void RigidBody::calculateAuxiliaries() {
    calculateV();
    calculateIinverse();
    calculateOmega();
}

