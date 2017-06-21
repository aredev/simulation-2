//
// Created by Tomirio on 21-6-2017.
//

#include "Solver.h"
#include "../forces/Force.h"
#include "../constraints/LambdaSolver.h"

void Solver:: applyForces(std::vector<Force *> forces) {
    for (auto &force : forces) {
        force->computeForce();
    }
}

void Solver::step(std::vector<Particle *> particles, std::vector<Force *> forces, std::vector<ConstraintForce *> constraints){
    // Clear forces
    Force::clearForces(particles);
    // Apply forces
    applyForces(forces);
    // Constraints
    LambdaSolver::solve(particles, constraints, 60, 5);
}
