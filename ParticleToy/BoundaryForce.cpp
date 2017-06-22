//
// Created by abdullah on 21/06/2017.
//

#include "BoundaryForce.h"
#include "utility.h"

BoundaryForce::BoundaryForce(std::vector<Particle *> p, std::vector<Vec2f> const &bc ) {
    this->particles = p;
    this->boundaries = bc;
}

BoundaryForce::BoundaryForce() : Force() {

}

void BoundaryForce::computeForce() {
    printf("%d \n", boundaries.size());
    for ( auto &particle : particles ){
        if (! boundaries.empty()){
            for ( auto &boundary : boundaries ){
                if ( utility::euclideanDistance(particle->m_Position, boundary) <= epsilon ){
                    //Too close to boundary, exert force
                    particle->m_Velocity -= Vec2f(particle->m_Velocity[0] + 0.1f, 0.0);
                    particle->m_Force -= Vec2f(particle->m_Force[0] + 0.1f, 0.0);
                }
            }
        }
    }
}

void BoundaryForce::draw() {
    Force::draw();
}
