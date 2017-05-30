//
// Created by Tomirio on 23-5-2017.
//

#include <vector>
#include "Particle.h"
#include "RK4.h"
#include "Force.h"
#include "ConstraintForce.h"
#include "LambdaSolver.h"

void applyForces(std::vector<Force *> forces){
        for(auto &force : forces){
            force->computeForce();
        }
    }

    void
    RK4::evaluate(std::vector<Particle *> particles, std::vector<Force *> forces, std::vector<ConstraintForce* > constraints,
             float dt) {
        Force::clearForces(particles);
        std::vector<Vec2f> orgPositions;
        // Apply forces
        applyForces(forces);

        // Remove the previously saved positions
        orgPositions.clear();
        // Save current positions as we need them in every evaluation
        for (auto &particle: particles) {
            orgPositions.push_back(particle->m_Position);
        }
        // Vectors for storing intermediate values
        std::vector<Vec2f> k1s;
        std::vector<Vec2f> k2s;
        std::vector<Vec2f> k3s;
        std::vector<Vec2f> k4s;
        // Counter
        unsigned i = 0;
        // Calculate k1's
        for (auto &particle: particles) {
            particle->m_Velocity += particle->force * dt;
            k1s.push_back(particle->force * dt);
        }

        // Clear and accumulate forces, apply constraints
        Force::clearForces(particles);
        // Apply forces
        applyForces(forces);

        // constraints
        LambdaSolver::solve(particles, constraints, 60, 5);

        // Calculate k2's
        for (auto &particle: particles) {
            particle->m_Position += orgPositions[i] + particle->m_Velocity * dt / 2.0f;
            particle->m_Velocity += particle->force * dt;
            k2s.push_back(particle->force + k1s[i] / 2.0f);
            i++;
        }

        // Clear and accumulate forces, apply constraints
        i = 0;
        Force::clearForces(particles);
        // Apply forces
        applyForces(forces);

        // constraints
        LambdaSolver::solve(particles, constraints, 60, 5);

        // Calculate k3's
        for (auto &particle: particles) {
            particle->m_Position += orgPositions[i] + particle->m_Velocity * dt / 2.0f;
            particle->m_Velocity += dt / 2.0f;
            k3s.push_back(particle->force + k2s[i] / 2.0f);
            i++;
        }

        // Clear and accumulate forces, apply constraints
        i = 0;
        Force::clearForces(particles);
        // Apply forces
        applyForces(forces);


        // constraints
        LambdaSolver::solve(particles, constraints, 60, 5);

        // Calculate k4's and do the final evaluation using the original position
        for (auto &particle: particles) {
            particle->m_Position += orgPositions[i] + particle->m_Velocity * dt;
            particle->m_Velocity += dt / 2.0f;
            k4s.push_back(particle->force + k3s[i]);
            particle->m_Position = orgPositions[i] + k1s[i] / 6.0f + k2s[i] / 3.0f + k3s[i] / 3.0f + k4s[i] / 6.0f;
            i++;
        }

    }

