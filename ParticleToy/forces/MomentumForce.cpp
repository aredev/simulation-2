//
// Created by Tomirio on 21-6-2017.
//

#include "MomentumForce.h"
#include "../Macros.h"

MomentumForce::MomentumForce(int N, float *mass, float *v, float *is_polygon_edge)
        : Force(), N(N), m_mass(mass), m_v(v), m_is_polygon_edge(is_polygon_edge) {

}


void MomentumForce::draw() {
    Force::draw();
}

// TODO we assume we only have one rigid body
void MomentumForce::computeForce() {
    int i, j;
    float linearMomentum = 0;
    FOR_EACH_CELL
            if (m_is_polygon_edge[IX(i, j)]) {
                float mass = m_mass[IX(i, j)];
                float veloc = m_v[IX(i, j)];
                linearMomentum += mass * veloc;
            }
    END_FOR
    particles[0]->m_Force += linearMomentum;
}
