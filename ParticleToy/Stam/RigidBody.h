//
// Created by Tomirio on 19-6-2017.
//

#ifndef SIMULATION_1_RIGIDBODY_H
#define SIMULATION_1_RIGIDBODY_H


#include <utility>
#include <Eigen/StdVector>
#include "../Particle.h"

using namespace Eigen;

class RigidBody : public Particle {
public:
    RigidBody();

    RigidBody(float *is_polygon_edge, float *mass, float *v, float *u_prev, float *v_prev, int N);


    // Functions
    void calculateCenterOfMass();

    void calculateTorque();

    void calculateAuxiliaries();

    void calculateAngularVelocity();

    void calculateMomentumOfIntertia();

    // Pointers for cell information
    float *m_is_polygon_edge;
    float *m_mass;
    float *m_v;
    float *u_prev;
    float *v_prev;
    const int N;

    /* State variables */
    Vector2f x;
    /* See slide 18 of cs685-rbm.pdf */
    Vector2f Rdot; /* R.t */
    Matrix2f R;
    Vector2f P, /* P.t/ */
            L; /* L.t/ */

    /* Derived quantities (auxiliary variables) */
    float I;  /* I−1.t/ */
    Vector2f omega;
    /* Computed quantities */
    Vector2f torque; /* .t/ */
};


#endif //SIMULATION_1_RIGIDBODY_H
