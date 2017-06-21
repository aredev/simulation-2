#ifndef SIMULATION_1_MIDPOINT_H
#define SIMULATION_1_MIDPOINT_H

#endif //SIMULATION_1_MIDPOINT_H

#include <vector>
#include "Solver.h"

class Midpoint : public Solver{
public:
    void evaluate(std::vector<Particle *> particles, std::vector<Force *> forces, std::vector<ConstraintForce *> constraints,
             float dt);
};