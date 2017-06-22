//
// Created by abdullah on 21/06/2017.
//

#ifndef SIMULATION_1_BOUNDARYFORCE_H
#define SIMULATION_1_BOUNDARYFORCE_H


#include "Force.h"

class BoundaryForce : public Force{

public:
    BoundaryForce();


    BoundaryForce(std::vector<Particle *> p, std::vector<Vec2f> &bc);


    BoundaryForce(std::vector<Particle *> p, std::vector<Vec2f> *bc);

    void computeForce() override;

    void draw() override;

    std::vector<Particle *> particles;
    std::vector<Vec2f> *boundaries;
    float epsilon = 0.025f;

};


#endif //SIMULATION_1_BOUNDARYFORCE_H
