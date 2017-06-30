//
// Created by Tomirio on 27-6-2017.
//

#ifndef SIMULATION_1_PARTICLESYSTEM_H
#define SIMULATION_1_PARTICLESYSTEM_H

#include <vector>
#include "Particle.h"
#include "../forces/ConstraintForce.h"
#include "RigidBody.h"

class ParticleSystem {
public:
    ParticleSystem();

    void draw();

    std::vector<Particle *> particles;

    std::vector<RigidBody *> rigidParticles;

    void clearForces();

    void applyForces();

    void solveConstraints();

    std::vector<Force *> forces;

    std::vector<ConstraintForce *> constraints;

    float t;

    void drawParticles();

    void drawConstraints();

    void drawForces();

    void drawVelocities();

    void printParticleGridIndices();
};


#endif //SIMULATION_1_PARTICLESYSTEM_H
