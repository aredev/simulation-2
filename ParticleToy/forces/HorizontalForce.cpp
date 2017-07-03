#include "HorizontalForce.h"

HorizontalForce::HorizontalForce(Particle* p1, double force) : Force() {

    this->p1 = p1;
    this->force = force;

}

void HorizontalForce::computeForce() {
    this->p1->m_Force += Vec2f(force, 0);
}

void HorizontalForce::draw() {
    Force::draw();
}
