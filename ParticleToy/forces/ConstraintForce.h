#ifndef SIMULATION_1_CONSTRAINTFORCE_H
#define SIMULATION_1_CONSTRAINTFORCE_H


#include "Force.h"
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class ConstraintForce : public Force {

public:
    ConstraintForce();

    virtual double getC();

    virtual double getCdot();

    virtual VectorXd getq(); //Get q, which is a vector of positions
    virtual MatrixXd getJ(); //Get J, which is the partial derivative vector
    virtual MatrixXd getW();

    virtual MatrixXd getJDot();

    virtual VectorXd getqDot();

    virtual VectorXd getQ();

    virtual void draw();

};


#endif //SIMULATION_1_CONSTRAINTFORCE_H
