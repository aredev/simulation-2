//
// Created by abdullah on 21/06/2017.
//

#include "BoundaryForce.h"
#include "../utility.h"
#include "../Macros.h"

BoundaryForce::BoundaryForce(std::vector<RigidBody *> &rigidParticles, float *u, float *v, float *dens, int N)
        : Force(), rigidParticles(rigidParticles), u(u), v(v), dens(dens), N(N) {
}


void BoundaryForce::computeForce() {
    float h = 1.0f / N;
    for (auto &rigidParticle : rigidParticles) {
        for (auto &polyCell : rigidParticle->polyEdges) {
            int i, j;
            float x, y;
            float polyCellX, polyCellY;
            Vec2f cellPos, polyCellPos;
            polyCellX = (polyCell[0] - 0.5f) * h;
            polyCellY = (polyCell[1] - 0.5f) * h;
            polyCellPos = Vec2f(polyCellX, polyCellY);
            FOR_EACH_CELL
                    if (dens[IX(i, j)] > 0) {
                        x = (i - 0.5f) * h;
                        y = (j - 0.5f) * h;
                        cellPos = Vec2f(x, y);
                        if (utility::euclideanDistance(cellPos, polyCellPos) <= epsilon) {
                            //Too close to boundary, exert force
                            v[IX(i, j)] *= -1;
                            u[IX(i, j)] *= -1;
                        }
                    }
            END_FOR
        }
    }
}

void BoundaryForce::draw() {
    Force::draw();
}