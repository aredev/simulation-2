#include "Solver.h"

Solver::Solver() = default;

unsigned int Solver::particleDims(ParticleSystem *p) {
    return 4 * p->particles.size();
}

void Solver::particleGetState(ParticleSystem *p, vector<float> &dst) {
    for (int i = 0; i < p->particles.size(); i++) {
        dst[i * 4] = p->particles[i]->m_Position[0];
        dst[i * 4 + 1] = p->particles[i]->m_Position[1];
        dst[i * 4 + 2] = p->particles[i]->m_Velocity[0];
        dst[i * 4 + 3] = p->particles[i]->m_Velocity[1];
    }
}

void Solver::particleSetState(ParticleSystem *p, vector<float> &src) {
    for (int i = 0; i < p->particles.size(); i++) {
        p->particles[i]->m_Position[0] = src[i * 4];
        p->particles[i]->m_Position[1] = src[i * 4 + 1];
        p->particles[i]->m_Velocity[0] = src[i * 4 + 2];
        p->particles[i]->m_Velocity[1] = src[i * 4 + 3];
    }
}

void Solver::particleDerivative(ParticleSystem *p, vector<float> &dst) {
    p->clearForces();
    p->applyForces();
    p->solveConstraints();
    for (int i = 0; i < p->particles.size(); i++) {
        /* xdot = v */
        dst[i * 4] = p->particles[i]->m_Velocity[0];
        dst[i * 4 + 1] = p->particles[i]->m_Velocity[1];
        /* vdot = f / m */
        float m = p->particles[i]->m_Mass;
        dst[i * 4 + 2] = p->particles[i]->m_Force[0] / m;
        dst[i * 4 + 3] = p->particles[i]->m_Force[1] / m;
    }
}

void Solver::scaleVector(vector<float> &src, float scalar) {
    for (float &i : src) {
        i *= scalar;
    }
}

void Solver::addVectors(vector<float> &v1, vector<float> &v2, vector<float> &dst) {
    for (int i = 0; i < v1.size(); i++) {
        dst[i] = v1[i] + v2[i];
    }
}

void Solver::scaleVector(vector<float> &src, float scalar, vector<float> &dst) {
    for (int i = 0; i < src.size(); i++) {
        dst[i] = src[i] * scalar;
    }
}


