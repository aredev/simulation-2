#ifndef SIMULATION_1_HORIZONTALFORCE_H
#define SIMULATION_1_HORIZONTALFORCE_H


#include "Force.h"

class HorizontalForce : public Force{
public:
    HorizontalForce(Particle *p1);

    HorizontalForce(Particle *p1, double force);

    void computeForce() override;

    void draw() override;

private:
    double force;
    Particle* p1;

};


#endif //SIMULATION_1_HORIZONTALFORCE_H
