//
// Created by Tomirio on 23-5-2017.
//

#include <vector>
#include "Solver.h"
#include "../constraints/ConstraintForce.h"
#include "../forces/Force.h"

#ifndef SIMULATION_1_FORWARDEULER_H
#define SIMULATION_1_FORWARDEULER_H

#endif //SIMULATION_1_FORWARDEULER_H

class ForwardEuler : public Solver{
public:

    void
    evaluate(std::vector<Particle *> particles, std::vector<Force *> forces, std::vector<ConstraintForce *> constraints,
             float dt);
};

