#pragma once

#include "Particle.h"
#include "ConstraintForce.h"

class RodConstraint : public ConstraintForce {
public:
    RodConstraint(Particle *p1, Particle *p2, double dist);

    Vec2f computeForce(Particle *p) override;

    void draw();

private:

    Particle *const m_p1;
    Particle *const m_p2;
    double const m_dist;
};
