#pragma once

#include "../particles/Particle.h"
#include "ConstraintForce.h"

class CircularWireConstraint : public ConstraintForce {
public:
    CircularWireConstraint(Particle *p, const Vec2f &center, const double radius);

    void draw();

    double getC() override;

    double getCdot() override;

    VectorXd getq() override;

    MatrixXd getJ() override;

    MatrixXd getW() override;

    MatrixXd getJDot() override;

    VectorXd getqDot() override;

    VectorXd getQ() override;

private:

    Particle *const m_p;
    Vec2f const m_center;
    double const m_radius;
};
