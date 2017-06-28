//
// Created by Tomirio on 19-6-2017.
//

#ifndef SIMULATION_1_RIGIDBODY_H
#define SIMULATION_1_RIGIDBODY_H


#include <utility>
#include <Eigen/StdVector>
#include "Particle.h"

using namespace Eigen;
using namespace std;

class RigidBody : public Particle {
public:
    RigidBody();
    RigidBody(float *mass, float *v, float *u_prev, float *v_prev, vector<Vec2f> &polyPoints, int N);

    // Pointers for cell information
    float *m_mass;
    float *m_v;
    float *u_prev;
    float *v_prev;

    // The points of the convex hull of the polygon
    vector<Vec2f> polyPoints;

    /*
    This vector contains the positions, which transformed to the index of the grid, indicating which cells
    the convex hull of the rigid body covers.
    */
    vector<Vec2f> *polyCells;


    int N;

    /* State variables */
    Vector2f x;
    Vector2f Rdot; /* R.t */
    Matrix2f R;
    Vector2f P, /* P.t/ */
            L;  /* L.t/ */

    // Functions

    void rasterizePolyEdges();

    void calculateTorque();

    void calculateAuxiliaries();

    void calculateAngularVelocity();

    void calculateMomentumOfIntertia();

private:
    void calculateCenterOfMass();
    void BresenhamLineAlgorithm(float x1, float y1, float x2, float y2, vector<Vec2f> &polyCells );

};


#endif //SIMULATION_1_RIGIDBODY_H
