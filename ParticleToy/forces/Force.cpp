#include "Force.h"
#include "../Stam/RigidBody.h"

Force::Force() {}

void Force::computeForce() {
    return;
}

void Force::draw() {
    return;
}

void Force::clearForces(std::vector<Particle *> particles) {
    for (auto &particle: particles) {
        particle->m_Force = Vec2f(0, 0);
    }
}