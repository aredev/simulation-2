#include <vector>
#include "ForwardEuler.h"

using namespace std;

void ForwardEuler::simulationStep(ParticleSystem *p, float dt) {
    vector<float> tmp1(this->particleDims(p)), tmp2(this->particleDims(p));
    particleDerivative(p, tmp1);
    scaleVector(tmp1, dt);
    particleGetState(p, tmp2);
    addVectors(tmp1, tmp2, tmp2);
    particleSetState(p, tmp2);
    p->t += dt;
}
