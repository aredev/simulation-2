#include <vector>
#include "../particles/Particle.h"
#include "RK4.h"
#include "../constraints/LambdaSolver.h"


void RK4::simulationStep(ParticleSystem *p, float dt) {
    unsigned int particleDimension = this->particleDims(p);

    vector<float> tmp(particleDimension), mid(particleDimension), org(particleDimension),
    k1(particleDimension), k2(particleDimension), k3 (particleDimension), k4(particleDimension);

    // Store original state
    particleGetState(p, org);

    // Calculate derivative of the particles, store in k1
    particleDerivative(p, tmp);
    // Multiple with half of the time step
    scaleVector(tmp, dt * 0.5f);
    // Add vectors
    addVectors(org, tmp, mid);
    // Set new state
    particleSetState(p, mid);
    // Save k1
    scaleVector(tmp, 2.0f, k1);

    // Calculate derivative
    particleDerivative(p, tmp);
    // Multiply with time step
    scaleVector(tmp, dt * 0.5f);
    // Add vectors
    addVectors(org, tmp, mid);
    // Set new state
    particleSetState(p, mid);
    // Save k2
    scaleVector(tmp, 2.0f, k2);

    // Calculate derivative
    particleDerivative(p, k3);
    // Multiply with time step
    scaleVector(k3, dt);
    // Add vectors
    addVectors(org, tmp, mid);
    // Set new state
    particleSetState(p, mid);

    // Calculate derivative
    particleDerivative(p, k4);
    scaleVector(k4, dt);

    // Final steps
    scaleVector(k1, 1.0f/6.0f);
    scaleVector(k2, 1.0f/3.0f);
    scaleVector(k3, 1.0f/3.0f);
    scaleVector(k4, 1.0f/6.0f);

    addVectors(org, k1, org);
    addVectors(org, k2, org);
    addVectors(org, k3, org);
    addVectors(org, k4, org);

    particleSetState(p, org);

    p->t += dt;
}
