//
// Created by abdullah on 19/06/2017.
//

#ifndef SIMULATION_1_SOLIDFLUIDFORCE_H
#define SIMULATION_1_SOLIDFLUIDFORCE_H


#include "Force.h"
#include "Stam/Marker.h"

class SolidFluidForce : public Force {

public:
    SolidFluidForce();

    SolidFluidForce(std::vector<Particle *> solids, Vec2f (&F_marks)[64*64], float * u, float * v, float* dens);


    SolidFluidForce(std::vector<Particle *> solids, float *u, float *v, float *u_add, float *v_add, float *dens);

    void computeForce() override;

    std::vector<Particle *> particles;

    float *u;
    float *v;
    float *u_add;
    float *v_add;
    float* dens;
};


#endif //SIMULATION_1_SOLIDFLUIDFORCE_H
