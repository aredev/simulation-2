#include <vector>
#include "../particles/Particle.h"
#include "../forces/Force.h"
#include "../forces/ConstraintForce.h"
#include "Solver.h"

#ifndef SIMULATION_1_RUNGEKUTTAFOURTH_H
#define SIMULATION_1_RUNGEKUTTAFOURTH_H


// During this evaluation, we will overwrite the velocities for the particles
// Therefore, we have to save them now so we can restore them later.
// We also need to clear this vector before we evaluate.

class RK4 : public Solver {
public:
    void simulationStep(ParticleSystem *p, float dt) override;
};

#endif //SIMULATION_1_RUNGEKUTTAFOURTH_H