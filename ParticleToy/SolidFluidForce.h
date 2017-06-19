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


    SolidFluidForce(std::vector<Particle *> solids, std::vector<Marker *> *markers, float kint, float zint, float dint,
                        float *u, float *v, float *dens);

    void computeForce() override;

    std::vector<Particle *> particles;
    std::vector<Marker *>* markers;
    float zint;
    float kint;
    float dint;

    float *u;
    float *v;
    float* dens;
};


#endif //SIMULATION_1_SOLIDFLUIDFORCE_H
