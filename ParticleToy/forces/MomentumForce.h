//
// Created by Tomirio on 21-6-2017.
//

#ifndef SIMULATION_1_MOMENTUMFORCE_H
#define SIMULATION_1_MOMENTUMFORCE_H


#include "Force.h"

class MomentumForce : public Force {
    MomentumForce(int N, float *mass, float *v, float *is_polygon_edge);
    void computeForce();
    void draw();

    float * m_is_polygon_edge;
    float * m_mass;
    float * m_v;
    int N;
};


#endif //SIMULATION_1_MOMENTUMFORCE_H
