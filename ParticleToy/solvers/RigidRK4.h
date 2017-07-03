//
// Created by Tomirio on 28-6-2017.
//

#ifndef SIMULATION_2_RIGIDRK4_H
#define SIMULATION_2_RIGIDRK4_H


#include "RK4.h"

class RigidRK4 : public RK4 {
public:
    int N;

    unsigned int particleDims(ParticleSystem *p) override;

    void particleGetState(ParticleSystem *p, vector<float> &dst) override;

    void particleSetState(ParticleSystem *p, vector<float> &src) override;

    void particleDerivative(ParticleSystem *p, vector<float> &dst) override;

};


#endif //SIMULATION_2_RIGIDRK4_H
