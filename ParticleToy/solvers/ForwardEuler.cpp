#include <vector>
#include "../Particle.h"
#include "../forces/Force.h"
#include "../constraints/ConstraintForce.h"
#include "../constraints/LambdaSolver.h"
#include "ForwardEuler.h"

void ForwardEuler::evaluate(std::vector<Particle *> particles, std::vector<Force *> forces,
                            std::vector<ConstraintForce *> constraints, float dt) {
    step(particles, forces, constraints);

    // Apply changes in velocity
    for (auto &particle: particles) {
        particle->m_Velocity += ( (particle->m_Force/particle->m_Mass) * dt);
        particle->m_Position += (particle->m_Velocity * dt);
    }

}