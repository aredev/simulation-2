//
// Created by abdullah on 20/06/2017.
//

#ifndef SIMULATION_1_COMPUTEFLUIDFORCE_H
#define SIMULATION_1_COMPUTEFLUIDFORCE_H

#include "../include/gfx/vec2.h"

class ComputeFluidForce {
    void computeMarkersInCells(Vec2f F_inters[64][64], int N);
};


#endif //SIMULATION_1_COMPUTEFLUIDFORCE_H
