//
// Created by Tomirio on 21-6-2017.
//

#ifndef SIMULATION_1_RIGIDFORWARDEULER_H
#define SIMULATION_1_RIGIDFORWARDEULER_H


#include <Eigen/StdVector>
#include "Solver.h"
#include "../Particle.h"
#include "../forces/Force.h"
#include "../constraints/ConstraintForce.h"
#include "../Stam/RigidBody.h"

class RigidForwardEuler : public Solver{
        public:
        void
        evaluate(std::vector<Particle *> particles, std::vector<Force *> forces, std::vector<ConstraintForce* > constraints, float dt);
};


#endif //SIMULATION_1_RIGIDFORWARDEULER_H
