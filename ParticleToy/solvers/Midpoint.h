#ifndef SIMULATION_1_MIDPOINT_H
#define SIMULATION_1_MIDPOINT_H

#endif //SIMULATION_1_MIDPOINT_H

#include <vector>
#include "../particles/Particle.h"
#include "../forces/Force.h"
#include "../forces/ConstraintForce.h"
#include "Solver.h"

class Midpoint : public Solver {
public:
    void simulationStep(ParticleSystem *p, float dt) override;
};