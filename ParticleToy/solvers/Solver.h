//
// Created by Tomirio on 21-6-2017.
//

#ifndef SIMULATION_1_SOLVER_H
#define SIMULATION_1_SOLVER_H


#include "../constraints/ConstraintForce.h"

class Solver {
public:
    Solver() {}

    virtual ~Solver() {}

    virtual void evaluate(std::vector<Particle *> particles, std::vector<Force *> forces,
                          std::vector<ConstraintForce *> constraints, float dt) = 0;

    static void
    step(std::vector<Particle *> particles, std::vector<Force *> forces, std::vector<ConstraintForce *> constraints);

private:
    static void applyForces(std::vector<Force *> forces);
};


#endif //SIMULATION_1_SOLVER_H
