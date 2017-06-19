//
// Created by Tomirio on 19-6-2017.
//

#include "RigidBody.h"
#include "Macros.h"
#include "../Particle.h"


using Eigen::MatrixXd;
using Eigen::VectorXd;


/**
 * Each particle has a constant location in the rigid body space.
 * This is denoted as r_{0i}.
 * The location of the ith particle in world space at time t, denoted ri.t is therefore given by
 * the formula: ri(t)= R(t)*r0i + x(t) where:
 * - R is the rotation matrix
 * - x(t) is the current location of the particle
 */


//


RigidBody::RigidBody(float *is_polygon_edge, float *mass, float *v, int N) : Particle(Vec2f(0, 0), 1),
                                                                             m_is_polygon_edge(is_polygon_edge),
                                                                             m_mass(mass), m_v(v),
                                                                             N(N) {
    m_R = MatrixXd::Zero(2, 2);
    calculateCenterOfMass();
}


void RigidBody::calculateCenterOfMass() {
    float totalMass = 0;
    MatrixXd massPositionSumation = MatrixXd::Zero(2, 1);
    int i, j;
    FOR_EACH_CELL
            if (m_is_polygon_edge[IX(i, j)]) {
                float mass = m_mass[IX(i, j)];
                totalMass += mass;
                MatrixXd pos = MatrixXd::Zero(2, 1);
                pos[0] = i - 1 / 2f * 1 / N;
                pos[1] = j - 1 / 2f * 1 / N;
                massPositionSumation += mass * pos;
            }
    END_FOR
    m_massOfBody = totalMass;
    m_centerOfMass = totalMass * massPositionSumation;
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
                pos[0] = i - 1 / 2f * 1 / N;
                pos[1] = j - 1 / 2f * 1 / N;
                float interia2D = mass * (pos[0] * pos[0] + pos[1] * pos[1]);
                // TODO we need the angular velocity. It can be calculated as follows: omega = I^(-1) * L
            }
    END_FOR
//    for (float individualMass : m_bodyMasses) {
//        MatrixXd r = m_R * m_constantBodyLocations[i] + m_currentBodyLocations[i];
//        MatrixXd tau = (r - m_centerOfMass); // * m_F[i] TODO multiply with force applied on the specific particle
//    }
}

// TODO look at the paper and the provided c++ code

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
    m_P = linearMomentum;
}


