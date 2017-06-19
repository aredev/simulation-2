//
// Created by Tomirio on 19-6-2017.
//

#ifndef SIMULATION_1_RIGIDBODY_H
#define SIMULATION_1_RIGIDBODY_H


#include <utility>
#include <Eigen/StdVector>
#include "../Particle.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class RigidBody : public Particle{
public:
    RigidBody();
    RigidBody(float *is_polygon_edge, float *mass, float *v, int N);
    void calculateCenterOfMass();
    void calculateTorque();
    void calculateLinearMomentum();

private:
    float m_massOfBody;
    VectorXd m_centerOfMass;
    VectorXd m_centerOfMassVelocity;
    float * m_is_polygon_edge;
    float * m_mass;
    float * m_v;
    const int N;
    MatrixXd m_R;
    int m_P;
    MatrixXd m_Ibody, m_IbodyInv;
    /* Derived quantities (auxiliary variables) */
    MatrixXd iInv;
};


#endif //SIMULATION_1_RIGIDBODY_H
