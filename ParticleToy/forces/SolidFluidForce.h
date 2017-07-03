//
// Created by abdullah on 19/06/2017.
//

#ifndef SIMULATION_1_SOLIDFLUIDFORCE_H
#define SIMULATION_1_SOLIDFLUIDFORCE_H


#include "Force.h"
#include "../particles/RigidBody.h"

class SolidFluidForce : public Force {

public:
    SolidFluidForce(std::vector<RigidBody *> &rigidBodies, float *u, float *v, float *u_add, float *v_add, float *dens, int N);

    void computeForce() override;

    std::vector<RigidBody *> rigidParticles;

    float *u;
    float *v;
    float *u_add;
    float *v_add;
    float *dens;
    int N;
};


#endif //SIMULATION_1_SOLIDFLUIDFORCE_H