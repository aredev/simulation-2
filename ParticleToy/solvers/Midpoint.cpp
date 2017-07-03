#include <vector>
#include "../forces/ConstraintForce.h"
#include "Midpoint.h"

void Midpoint::simulationStep(ParticleSystem *p, float dt) {
    unsigned int particleDimension = this->particleDims(p);
    vector<float> tmp1(particleDimension), tmp2(particleDimension), tmp3(particleDimension);
    // Store starting point
    particleGetState(p, tmp2);
    // Calculate derivative of the particles
    particleDerivative(p, tmp1);
    // Take a step towards the midpoint
    scaleVector(tmp1, dt * 0.5f);
    // Add starting point to mid-point, and store
    addVectors(tmp1, tmp2, tmp2);
    // Save starting point
    particleGetState(p, tmp3);
    // Set to newly calculated state
    particleSetState(p, tmp2);
    // Calculate derivative of the particles
    particleDerivative(p, tmp2);
    // Take a time step
    scaleVector(tmp2, dt);
    // Add to starting point
    addVectors(tmp2, tmp3, tmp3);
    // Set new state
    particleSetState(p, tmp3);
    // Increment times
    p->t += dt;
}
