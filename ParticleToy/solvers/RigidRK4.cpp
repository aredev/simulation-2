//
// Created by Tomirio on 28-6-2017.
//

#include "RigidRK4.h"
#include "../Macros.h"

using namespace Eigen;


unsigned int RigidRK4::particleDims(ParticleSystem *p) {
    return 10 * p->rigidParticles.size();
}

void RigidRK4::particleGetState(ParticleSystem *p, vector<float> &dst) {
    for (int i = 0; i < p->rigidParticles.size(); i++) {
        dst[i * 10] = p->rigidParticles[i]->m_Position[0];
        dst[i * 10 + 1] = p->rigidParticles[i]->m_Position[1];
        dst[i * 10 + 2] = p->rigidParticles[i]->R(0, 0);
        dst[i * 10 + 3] = p->rigidParticles[i]->R(0, 1);
        dst[i * 10 + 4] = p->rigidParticles[i]->R(1, 0);
        dst[i * 10 + 5] = p->rigidParticles[i]->R(1, 1);
        dst[i * 10 + 6] = p->rigidParticles[i]->P(0);
        dst[i * 10 + 7] = p->rigidParticles[i]->P(1);
        dst[i * 10 + 8] = p->rigidParticles[i]->L(0);
        dst[i * 10 + 9] = p->rigidParticles[i]->L(1);
    }
}

void RigidRK4::particleSetState(ParticleSystem *p, vector<float> &src) {
    for (int i = 0; i < p->rigidParticles.size(); i++) {
        p->rigidParticles[i]->m_Position[0] = src[i * 10];
        p->rigidParticles[i]->m_Position[1] = src[i * 10 + 1];
        p->rigidParticles[i]->R(0, 0) = src[i * 10 + 2];
        p->rigidParticles[i]->R(0, 1) = src[i * 10 + 3];
        p->rigidParticles[i]->R(1, 0) = src[i * 10 + 4];
        p->rigidParticles[i]->R(1, 1) = src[i * 10 + 5];
        p->rigidParticles[i]->P(0) = src[i * 10 + 6];
        p->rigidParticles[i]->P(1) = src[i * 10 + 7];
        p->rigidParticles[i]->L(0) = src[i * 10 + 8];
        p->rigidParticles[i]->L(1) = src[i * 10 + 9];


    }
}

void RigidRK4::particleDerivative(ParticleSystem *p, vector<float> &dst) {
    p->clearForces();
    p->applyForces();
    p->solveConstraints();
    for (unsigned int i = 0; i < p->rigidParticles.size(); i++) {
        // Compute auxiliary variables
        RigidBody *r = p->rigidParticles.at(i);
        r->calculateAuxiliaries();
        /* xdot = v */
        dst[i * 10] = p->rigidParticles[i]->m_Velocity[0];
        dst[i * 10 + 1] = p->rigidParticles[i]->m_Velocity[1];
        /* Rdot = omega * R */
        dst[i * 10 + 2] = p->rigidParticles[i]->R(0, 0) * p->rigidParticles[i]->omega;
        dst[i * 10 + 3] = p->rigidParticles[i]->R(0, 1) * p->rigidParticles[i]->omega;
        dst[i * 10 + 4] = p->rigidParticles[i]->R(1, 0) * p->rigidParticles[i]->omega;
        dst[i * 10 + 5] = p->rigidParticles[i]->R(1, 1) * p->rigidParticles[i]->omega;
        /* Pdot = F */
        dst[i * 10 + 6] = p->rigidParticles[i]->m_Force[0];
        dst[i * 10 + 7] = p->rigidParticles[i]->m_Force[1];
        /* Ldot = tau */
        dst[i * 10 + 8] = p->rigidParticles[i]->torque[0];
        dst[i * 10 + 9] = p->rigidParticles[i]->torque[1];
    }
}
