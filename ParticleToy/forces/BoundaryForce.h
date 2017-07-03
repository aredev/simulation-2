//
// Created by abdullah on 21/06/2017.
//

#ifndef SIMULATION_1_BOUNDARYFORCE_H
#define SIMULATION_1_BOUNDARYFORCE_H


#include "Force.h"
#include "../particles/RigidBody.h"

class BoundaryForce : public Force {

public:
    BoundaryForce(std::vector<RigidBody *> &rigidParticles, float *u, float *v, float *dens, int N);

    void computeForce() override;

    void draw() override;

    std::vector<RigidBody *> rigidParticles;
    float epsilon = 0.025f;

    float *u;
    float *v;
    float *dens;
    int N;

};


#endif //SIMULATION_1_BOUNDARYFORCE_H