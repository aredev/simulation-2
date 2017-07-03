//
// Created by Tomirio on 23-5-2017.
//

#include <vector>
#include "../particles/Particle.h"
#include "Solver.h"

#ifndef SIMULATION_1_FORWARDEULER_H
#define SIMULATION_1_FORWARDEULER_H

#endif //SIMULATION_1_FORWARDEULER_H

class ForwardEuler : public Solver {
public:
    void simulationStep(ParticleSystem *p, float dt) override;

};

