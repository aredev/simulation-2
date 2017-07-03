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
    RigidBody(float *mass, float *v, float *u_prev, float *v_prev, vector<Vec2f> &polyPoints, int N);

    RigidBody(float *mass, float *v, float *u_prev, float *v_prev, int N);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Pointers for cell information
    float *m_mass;
    float *m_v;
    float *u_prev;
    float *v_prev;

    // The points of the convex hull of the polygon
    vector<Vec2f> polyEdges;

    /*
    This vector contains the positions, which transformed to the index of the grid, indicating which cells
    the convex hull of the rigid body covers.
    */
    vector<Vec2f> polyEdgeCells;


    int N;

    /* State variables */
    Vector2f x;
    Matrix2f R;
    Vector2f P, /* P.t/ */
            L;  /* L.t/ */

    /* Auxilarry variables */
    Vector2f torque;
    float omega;
    float I;

    // Functions
    void draw() override;

    void drawPolyPoints();

    void drawPolyEdges();

    void rasterizePolyEdges();

    void calculateTorque();

    void calculateVelocity();

    void calculateAuxiliaries();

    void calculateAngularVelocity();

    void calculateMomentumOfIntertia();

    void printPolyPointsGridIndices();

private:
    void calculateCenterOfMass();

    void BresenhamLineAlgorithm(float x1, float y1, float x2, float y2);
};


#endif //SIMULATION_1_RIGIDBODY_H
