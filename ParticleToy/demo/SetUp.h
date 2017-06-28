#ifndef SIMULATION_1_SETUP_H
#define SIMULATION_1_SETUP_H


#include <Eigen/StdVector>
#include "../particles/Particle.h"
#include "../forces/Force.h"
#include "../constraints/ConstraintForce.h"

class SetUp {

public:
    static void setUpSpringforce(std::vector<Particle *> &pVector, std::vector<Force *> &forces);

    static void setUpGravity(std::vector<Particle *> &pVector, std::vector<Force *> &forces);

    static void setUpRodConstraint(std::vector<Particle *> &pVector, std::vector<Force *> &forces, std::vector<ConstraintForce *> &constraints);

    static void setUpCircularConstraint(std::vector<Particle *> &pVector, std::vector<Force *> &forces, std::vector<ConstraintForce *> &constraints);

    static void setUpHorizontalConstraint(std::vector<Particle *> &pVector, std::vector<Force *> &forces, std::vector<ConstraintForce *> &constraints);

    static void setUpMixedConstraint(std::vector<Particle *> &pVector, std::vector<Force *> &forces, std::vector<ConstraintForce *> &constraints);

    static void setUpSlidingCloth(std::vector<Particle *> &pVector, std::vector<Force *> &forces, std::vector<ConstraintForce *> &constraints, int size);

    static void setUpSlidingClothWall(std::vector<Particle *> &pVector, std::vector<Force *> &forces, std::vector<ConstraintForce *> &constraints, int size);

    static void setUpAngularSpring(std::vector<Particle *> &pVector, std::vector<Force *> &forces);
};


#endif //SIMULATION_1_SETUP_H
