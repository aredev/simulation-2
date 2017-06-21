#include "ConstraintForce.h"
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

ConstraintForce::ConstraintForce() {}

VectorXd ConstraintForce::getQ() {
    MatrixXd matrix(1,1);
    return matrix;
}

VectorXd ConstraintForce::getqDot() {
    MatrixXd matrix(1,1);
    return matrix;
}

VectorXd ConstraintForce::getq() {
    MatrixXd matrix(1,1);
    return matrix;
}

MatrixXd ConstraintForce::getJDot() {
    MatrixXd matrix(1,1);
    return matrix;
}

MatrixXd ConstraintForce::getJ() {
    MatrixXd matrix(1,1);
    return matrix;
}

MatrixXd ConstraintForce::getW() {
    MatrixXd matrix(1,1);
    return matrix;
}

double ConstraintForce::getCdot() {
    return {0};
}

double ConstraintForce::getC() {
    return 0.0;
}

void ConstraintForce::draw() {
    return;
}

