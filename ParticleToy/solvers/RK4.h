#include <vector>
#include "../Particle.h"
#include "../forces/Force.h"
#include "../constraints/ConstraintForce.h"
#include "Solver.h"

#ifndef SIMULATION_1_RUNGEKUTTAFOURTH_H
#define SIMULATION_1_RUNGEKUTTAFOURTH_H


// During this evaluation, we will overwrite the velocities for the particles
// Therefore, we have to save them now so we can restore them later.
// We also need to clear this vector before we evaluate.

class RK4 : public Solver{
public:
    void evaluate(std::vector<Particle *> particles, std::vector<Force *> forces, std::vector<ConstraintForce* > constraints,
             float dt);
};

#endif //SIMULATION_1_RUNGEKUTTAFOURTH_H