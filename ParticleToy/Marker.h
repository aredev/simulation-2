//
// Created by abdullah on 14/06/2017.
//

#ifndef SIMULATION_1_MARKER_H
#define SIMULATION_1_MARKER_H


#include "particles/Particle.h"

class Marker : public Particle{
public:
    Marker(const Vec2f &ConstructPos, double m);

    Marker(const Vec2f &ConstructPos, double m, double density, double velocity);

    void draw() override;

    double density;
    double velocity;

};


#endif //SIMULATION_1_MARKER_H
