#ifndef SIMULATION_1_GRAVITYFORCE_H
#define SIMULATION_1_GRAVITYFORCE_H


#include "Force.h"

class GravityForce : public Force {

public:
    GravityForce(std::vector<Particle *> p);

    GravityForce();

    void computeForce() override;
};


#endif //SIMULATION_1_GRAVITYFORCE_H
