#ifndef SIMULATION_1_WALLFORCE_H
#define SIMULATION_1_WALLFORCE_H


#include "Force.h"

class WallForce : public Force{

public:
    WallForce();

    WallForce(Particle *p1, double x);

    WallForce(Particle *p1, double x, double y);

    void computeForce() override;

    void draw() override;

private:
    double x;
    double y = -1.0f;
    Particle* p;
    float epsilon = 0.025f;

};


#endif //SIMULATION_1_WALLFORCE_H
