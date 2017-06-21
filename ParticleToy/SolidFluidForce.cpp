//
// Created by abdullah on 19/06/2017.
//

#include "SolidFluidForce.h"
#include "Stam/Marker.h"

#define N 64
#define IX(i,j) ((i)+(N+2)*(j))
#define FOR_EACH_CELL for ( i=1 ; i<=N ; i++ ) { for ( j=1 ; j<=N ; j++ ) {
#define END_FOR }}

SolidFluidForce::SolidFluidForce(std::vector<Particle *> solids, float * u, float * v, float * u_add, float * v_add, float* dens) : Force() {
    this->particles = solids;
    this->u = u;
    this->v = v ;
    this->dens = dens;
    this->u_add = u_add;
    this->v_add = v_add;
}


void SolidFluidForce::computeForce() {
    float kint = 0.0f;
    float dint = 0.01;
    float zint = 0.05f;
    Vec2f R = Vec2f(0.2, 0.2);

    int i, j;

    for (auto& solid: particles) {
        Vec2f fElem = Vec2f(0.0, 0.0);
        FOR_EACH_CELL
            float x, y, h;
            h = 1.0f / N;
            x = (i - 0.5f) * h;
            y = (j - 0.5f) * h;

            if (dens[IX(i,j)] > 0){
                float distance = sqrtf(
                        powf(solid->m_Position[0] - x, 2.0) +
                        powf(solid->m_Position[1] - y, 2.0)
                );

                if (distance < 0.05f) {
                    float xDist = fabsf(solid->m_Position[0] - x);
                    float yDist = fabsf(solid->m_Position[1] - y);

                    fElem += -1 * kint * ( Vec2f(xDist, yDist) - Vec2f(dint, dint) ) - (zint * ( Vec2f(u[IX(i,j)] * xDist, v[IX(i,j)] * yDist) ));
                }
            }

//          Add the force of the entity to the marker.

            Vec2f Fadjacent = Vec2f(0.0, 0.0);
            Vec2f lambda = Vec2f(0.0, 0.0);
            //If there is space to the right
            if (i+1 < N){
                //Use n = 0
                Fadjacent = Vec2f(u_add[IX(i + 1,j)], v_add[IX(i + 1,j)]); //Get the force of the element to the right
                lambda = Vec2f(1.0f, 0.0f); //Lambda is [delta_00, delta_10]
            }else if (j+1 < N){
                // Use n = 1
                Fadjacent = Vec2f(u_add[IX(i,j+1)], v_add[IX(i,j+1)]); //Get the force of the element to the top
                lambda = Vec2f(0.0f, 1.0f); //Lambda is [delta_01 , delta_11]
            }
            Vec2f addForce = ( fElem / 2 * lambda ) + ( Fadjacent / 2 * lambda );


            u_add[IX(i, j)] += addForce[0];
            v_add[IX(i, j)] += addForce[1];



        END_FOR
        solid->m_Force += fElem;
        solid->m_Velocity *= 0.99f;
        solid->m_Force *= 0.93f;
    }

}

