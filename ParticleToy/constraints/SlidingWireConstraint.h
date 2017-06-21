#ifndef SIMULATION_1_SLIDINGWIRECONSTRAINT_H
#define SIMULATION_1_SLIDINGWIRECONSTRAINT_H


#include "ConstraintForce.h"

class SlidingWireConstraint : public ConstraintForce{

public:
    SlidingWireConstraint();

    SlidingWireConstraint(Particle *p1, double height);

    double getC() override;

    double getCdot() override;

    VectorXd getq() override;

    MatrixXd getJ() override;

    MatrixXd getW() override;

    MatrixXd getJDot() override;

    VectorXd getqDot() override;

    VectorXd getQ() override;

    void draw() override;

private:
    double height;
    Particle * p1;

};


#endif //SIMULATION_1_SLIDINGWIRECONSTRAINT_H
