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

    RigidBody(float *is_polygon_edge, float *mass, float *v, int N);

    void calculateCenterOfMass();

    void calculateTorque();

    void calculateLinearMomentum();

    void calculateAuxiliaries();


private:
    // Functions
    void calculateV();

    void calculateIinverse();

    void calculateIbody();

    void calculateOmega();

    // Pointers for cell information
    float *m_is_polygon_edge;
    float *m_mass;
    float *m_v;
    const int N;

    /* Constant quantities */
    Matrix2f Ibody, /* Ibody */
            Ibodyinv; /* I−1 body (inverse of Ibody) */

    /* State variables */
    Vector2f x;
    Matrix2f R; /* R.t/ */
    Vector2f P, /* P.t/ */
            L; /* L.t/ */

    /* Derived quantities (auxiliary variables) */
    Matrix2f Iinv;  /* I−1.t/ */
    Vector2f v,     /* v.t/ */
            omega;  /* !.t/ */

    /* Computed quantities */
    Vector2f torque; /* .t/ */
};


#endif //SIMULATION_1_RIGIDBODY_H
