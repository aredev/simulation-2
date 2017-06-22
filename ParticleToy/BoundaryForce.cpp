//
// Created by abdullah on 21/06/2017.
//

#include "BoundaryForce.h"
#include "utility.h"

BoundaryForce::BoundaryForce(std::vector<Particle *> p, std::vector<Vec2f> *bc ) {
    this->particles = p;
    this->boundaries = bc;
}

BoundaryForce::BoundaryForce() : Force() {

}

void BoundaryForce::computeForce() {
    for ( auto &particle : particles ){
        for ( auto &boundary : *boundaries ){
            if ( utility::euclideanDistance(particle->m_Position, boundary) <= epsilon ){
                //Too close to boundary, exert force
                particle->m_Velocity *= -1;
                particle->m_Force *= -1;
            }
        }
    }
}

void BoundaryForce::draw() {
    Force::draw();
}
