#ifndef SIMULATION_1_FORCE_H
#define SIMULATION_1_FORCE_H


#include <vector>
#include "../Particle.h"

class Force {

public:

    Force();

    //Compute the force
    virtual void computeForce();

    static void clearForces(std::vector<Particle *> particles);

    std::vector<Particle *> particles;
    float gravitationalConstant;

    virtual void draw();

};


#endif //SIMULATION_1_FORCE_H
