//
// Created by Tomirio on 21-6-2017.
//

#include "RigidForwardEuler.h"
#include "../constraints/LambdaSolver.h"
#include "../Stam/RigidBody.h"

void RigidForwardEuler::evaluate(std::vector<Particle *> particles, std::vector<Force *> forces,
                                 std::vector<ConstraintForce *> constraints, float dt) {

    step(particles, forces, constraints);
    // We als apply the momentum force
    // Apply changes in velocity
    Vector2f tmpForce;
    for (auto particle: particles) {
        RigidBody *p = dynamic_cast<RigidBody *>(particle);
        p->calculateAuxiliaries();
        p->m_Velocity += ((p->m_Force / p->m_Mass) * dt);
        p->m_Position += (particle->m_Velocity * dt);
        p->Rdot += dt * (p->R * p->omega);
        tmpForce(0) = p->m_Force[0];
        tmpForce(1) = p->m_Force[1];
        p->P += dt * tmpForce;
        p->L += dt * p->torque;
    }
}

