//
// Created by Tomirio on 27-6-2017.
//

#ifndef SIMULATION_1_SOLVER_H
#define SIMULATION_1_SOLVER_H


#include "../particles/ParticleSystem.h"
#include "../forces/ConstraintForce.h"
#include <vector>

using namespace std;

class Solver {
public:
    Solver();

    virtual unsigned int particleDims(ParticleSystem *p);

    virtual void particleGetState(ParticleSystem *p, vector<float> &dst);

    virtual void particleSetState(ParticleSystem *p, vector<float> &src);

    virtual void particleDerivative(ParticleSystem *p, vector<float> &dst);

    virtual void simulationStep(ParticleSystem *p, float dt) = 0;

    void scaleVector(vector<float> &src, float scalar);

    void scaleVector(vector<float> &src, float scalar, vector<float> &dst);

    void addVectors(vector<float> &v1, vector<float> &v2, vector<float> &dst);
};


#endif //SIMULATION_1_SOLVER_H
