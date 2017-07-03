//
// Created by abdullah on 19/06/2017.
//

#include "SolidFluidForce.h"

#include"../Macros.h"

SolidFluidForce::SolidFluidForce(std::vector<RigidBody *> &rigidBodies, float *u, float *v, float *u_add, float *v_add,
                                 float *dens, int N) : u(u), v(v), u_add(u_add), v_add(v_add) {
}


void SolidFluidForce::computeForce() {
    float kint = 0.0f;
    float dint = 0.01;
    float zint = 0.05f;
    Vec2f R = Vec2f(0.2, 0.2);

    int i, j;
    float x, y, h;

    for (auto &rigidBody: rigidParticles) {
        Vec2f fElem = Vec2f(0.0, 0.0);
        for (auto &polyCell: rigidBody->polyEdgeCells) {
            h = 1.0f / N;
            x = (i - 0.5f) * h;
            y = (j - 0.5f) * h;

            i = polyCell[0];
            j = polyCell[1];

            if (dens[IX(i, j)] > 0) {
                float xDist = fabsf(rigidBody->m_Position[0] - x);
                float yDist = fabsf(rigidBody->m_Position[1] - y);

                fElem += -1 * kint * (Vec2f(xDist, yDist) - Vec2f(dint, dint)) -
                         (zint * (Vec2f(u[IX(i, j)] * xDist, v[IX(i, j)] * yDist)));
            }

//          Add the force of the entity to the marker. This ensures that the solid also exerts force on the fluid.

            Vec2f Fadjacent = Vec2f(0.0, 0.0);
            Vec2f lambda = Vec2f(0.0, 0.0);
            //If there is space to the right
            if (i + 1 < N) {
                //Use n = 0
                Fadjacent = Vec2f(u_add[IX(i + 1, j)],
                                  v_add[IX(i + 1, j)]); //Get the force of the element to the right
                lambda = Vec2f(1.0f, 0.0f); //Lambda is [delta_00, delta_10]
            } else if (j + 1 < N) {
                // Use n = 1
                Fadjacent = Vec2f(u_add[IX(i, j + 1)],
                                  v_add[IX(i, j + 1)]); //Get the force of the element to the top
                lambda = Vec2f(0.0f, 1.0f); //Lambda is [delta_01 , delta_11]
            }
            Vec2f addForce = (fElem / 2 * lambda) + (Fadjacent / 2 * lambda);

            u_add[IX(i, j)] += addForce[0] * 2;
            v_add[IX(i, j)] += addForce[1] * 2;

        }

        rigidBody->m_Force += fElem;
        rigidBody->m_Velocity *= 0.99f;
        rigidBody->m_Force *= 0.93f;
    }

}
