#include <vector>
#include "Particle.h"
#include "RK4.h"
#include "LambdaSolver.h"

void applyForces(std::vector<Force *> forces) {
    for (auto &force : forces) {
        force->computeForce();
    }
}

void
RK4::evaluate(std::vector<Particle *> particles, std::vector<Force *> forces,
              std::vector<ConstraintForce *> constraints,
              float dt) {
    // Vector for storing original positions
    std::vector<Vec2f> orgPositions;
    // Vectors for storing intermediate values
    std::vector<Vec2f> k1s;
    std::vector<Vec2f> k2s;
    std::vector<Vec2f> k3s;
    std::vector<Vec2f> k4s;

    // Counter, used for looping through the k's
    unsigned i = 0;

    // Save current positions as we need them in every evaluation
    for (auto &particle: particles) {
        orgPositions.push_back(particle->m_Position);
    }

    // Clear forces
    Force::clearForces(particles);
    // Apply forces
    applyForces(forces);
    // Constraints
    LambdaSolver::solve(particles, constraints, 60, 5);

    // Calculate k1's
    for (auto &particle: particles) {
        particle->m_Velocity += (particle->m_Force/particle->m_Mass) * dt;
        k1s.push_back(particle->m_Velocity * dt);
        particle->m_Position += particle->m_Velocity * dt / 2.0f;
    }

    i = 0;
    // Clear forces
    Force::clearForces(particles);
    // Apply forces
    applyForces(forces);
    // Constraints
    LambdaSolver::solve(particles, constraints, 60, 5);

    // Calculate k2's
    for (auto &particle: particles) {
        particle->m_Velocity += (particle->m_Force/particle->m_Mass) * dt;
        k2s.push_back(particle->m_Velocity * dt);
        particle->m_Position = orgPositions[i] + particle->m_Velocity * dt / 2.0f;
        i++;
    }

    i = 0;
    // Clear forces
    Force::clearForces(particles);
    // Apply forces
    applyForces(forces);
    // Constraints
    LambdaSolver::solve(particles, constraints, 60, 5);

    // Calculate k3's
    for (auto &particle: particles) {
        particle->m_Velocity += (particle->m_Force/particle->m_Mass) * dt;
        k3s.push_back(particle->m_Velocity * dt);
        particle->m_Position = orgPositions[i] + particle->m_Velocity * dt;
        i++;
    }

    i = 0;
    // Clear forces
    Force::clearForces(particles);
    // Apply forces
    applyForces(forces);
    // Constraints
    LambdaSolver::solve(particles, constraints, 60, 5);

    // Calculate k4's and do the final evaluation using the original position
    for (auto &particle: particles) {
        particle->m_Velocity += (particle->m_Force/particle->m_Mass) * dt;
        k4s.push_back(particle->m_Velocity * dt);
        particle->m_Position = orgPositions[i] + k1s[i] / 6.0f + k2s[i] / 3.0f + k3s[i] / 3.0f + k4s[i] / 6.0f;
        i++;
    }
    orgPositions.clear();
}

