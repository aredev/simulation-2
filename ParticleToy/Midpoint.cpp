#include <vector>
#include "Particle.h"
#include "Force.h"
#include "ConstraintForce.h"
#include "Midpoint.h"
#include "LambdaSolver.h"


void Midpoint::evaluate(std::vector<Particle *> particles, std::vector<Force *> forces,
                        std::vector<ConstraintForce *> constraints,
                        float dt) {
    unsigned i = 0;
    // Vector for storing the original positions
    std::vector<Vec2f> orgPositions;
    // Clear previous positions
    orgPositions.clear();
    // Clear forces
    Force::clearForces(particles);
    // Save current positions as we need them later on
    for (auto &particle: particles) {
        orgPositions.push_back(particle->m_Position);
    }
    // Apply forces
    for (auto &force: forces) {
        force->computeForce();
    }
    // Apply changes in velocity
    for (auto &particle: particles) {
        particle->m_Velocity += (particle->m_Force/particle->m_Mass) * dt;
        particle->m_Position += particle->m_Velocity * dt / 2.0f;
    }

    // Clear the forces
    Force::clearForces(particles);
    // Apply forces
    for (auto &force: forces) {
        force->computeForce();
    }
    // Constraints
    LambdaSolver::solve(particles, constraints, 60, 5);

    // Final evaluation
    for (auto &particle: particles) {
        particle->m_Velocity += particle->m_Force * dt / 2.0f;
        particle->m_Position = orgPositions[i++] + particle->m_Velocity * dt;
    }
}