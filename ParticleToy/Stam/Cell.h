//
// Created by abdullah on 14/06/2017.
//

#ifndef SIMULATION_1_CELL_H
#define SIMULATION_1_CELL_H

#include <gfx/vec2.h>
#include <Eigen/StdVector>
#include "../Particle.h"


class Cell {

public:
    Vec2f m_Position;
    Vec2f m_Density;
    float m_Pressure;
    float width;

    std::vector<Particle *> fluidParticles;


};


#endif //SIMULATION_1_CELL_H
