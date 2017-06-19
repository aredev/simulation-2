//
// Created by abdullah on 19/06/2017.
//

#include "SolidFluidForce.h"
#include "Stam/Marker.h"

#define N 64
#define IX(i,j) ((i)+(N+2)*(j))
#define FOR_EACH_CELL for ( i=1 ; i<=N ; i++ ) { for ( j=1 ; j<=N ; j++ ) {
#define END_FOR }}

SolidFluidForce::SolidFluidForce(std::vector<Particle *> solids, std::vector<Marker *> *markers, float kint, float zint, float dint, float * u, float * v, float* dens) : Force() {
    this->markers = markers;
    this->particles = solids;
    this->kint = kint;
    this->zint = zint;
    this->dint = dint;
    this->u = u;
    this->v = v ;
    this->dens = dens;
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
//                solid->m_Velocity = Vec2f(u[IX(i,j)], v[IX(i,j)]);
//                solid->m_Velocity *= 0.97;
                    float xDist = fabsf(solid->m_Position[0] - x);
                    float yDist = fabsf(solid->m_Position[1] - y);

                    fElem += -1 * kint * ( Vec2f(xDist, yDist) - Vec2f(dint, dint) ) - (zint * ( Vec2f(u[IX(i,j)] * xDist, v[IX(i,j)] * yDist) ));

//
                }
            }


        END_FOR
        solid->m_Force += fElem;
        solid->m_Velocity *= 0.99f;
        solid->m_Force *= 0.93f;
    }

//    printf("Markers %d \n", markers->size());
//    for (auto &solid: particles) {
//        Vec2f fElem = Vec2f(0, 0);
//        Vec2f vElem = Vec2f(0.0, 0.0);
//        for (int i = 0 ; i < markers->size(); i++) {
//
////            i = (int)((       marker->m_Position[0] /(float)512)*64+1);
////            j = (int)(((512-marker->m_Position[1])/(float)512)*64+1);
////
////            float x, y, h;
////            h = 1.0f / N;
////            x = (i - 0.5f) * h;
////            y = (j - 0.5f) * h;
//
//            Marker* marker = markers->at(i);
//
//            float distance = sqrtf(
//                    powf(solid->m_Position[0] - marker->m_Position[0], 2.0) +
//                    powf(solid->m_Position[1] - marker->m_Position[1], 2.0)
//            );
//
//            if (distance < 0.01f) {
//                vElem += marker->m_Velocity;
//            }
//
//
//        }
//        solid->m_Velocity = vElem;
//    }
}
