#ifndef SIMULATION_1_WALLFORCE_H
#define SIMULATION_1_WALLFORCE_H


#include "Force.h"

class WallForce : public Force{

public:
    WallForce();

    WallForce(Particle *p1, double x);

    void computeForce() override;

    void draw() override;

private:
    double x;
    Particle* p;

};


#endif //SIMULATION_1_WALLFORCE_H
