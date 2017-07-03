#include "LambdaSolver.h"
#include "Eigen/Dense"
#include "Eigen/IterativeLinearSolvers"
#include "ConstraintForce.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::ConjugateGradient;
using Eigen::VectorXd;

void assignRowsToParticles(std::vector<Particle *> particles){
    int row = 0;
    for (auto &particle : particles){
        particle->row = row;
        row+=2;
    }
}

void LambdaSolver::solve(std::vector<Particle *> particles, std::vector<ConstraintForce *> constraints, float ks, float kd ){
    MatrixXd J = MatrixXd::Zero(constraints.size(), particles.size()*2);
    MatrixXd Jdot = MatrixXd::Zero(constraints.size(), particles.size()*2);
    VectorXd C(constraints.size());
    VectorXd Cdot(constraints.size());

    assignRowsToParticles(particles);

    //Build the J matrix
    for ( int column = 0 ; column < constraints.size() ; column++ ) {
        ConstraintForce* constraint = constraints[column];
        for ( int particlePos = 0; particlePos < constraint->particles.size(); particlePos++ ) {
            //Set x value of particle in J
            Particle* particle = constraint->particles[particlePos];
            J(column, particle->row) = constraint->getJ()(0, particlePos*2);
            Jdot(column, particle->row) = constraint->getJDot()(0, particlePos*2);

            //Set y value of particle in J
            J(column, particle->row+1) = constraint->getJ()(0, particlePos*2+1);
            Jdot(column, particle->row+1) = constraint->getJDot()(0, particlePos*2+1);
        }

        C(column) = constraint->getC();
        Cdot(column) = constraint->getCdot();
    }

    MatrixXd JW = J;

    MatrixXd JWJT = JW * J.transpose();


    VectorXd qdot(particles.size() * 2);
    VectorXd Q(particles.size() * 2);
    //Build the qdot vector
    for (auto &particle : particles){
        qdot(particle->row) = particle->m_Velocity[0];
        qdot(particle->row+1) = particle->m_Velocity[1];

        Q(particle->row) = particle->m_Force[0];
        Q(particle->row+1) = particle->m_Force[1];
    }

    MatrixXd JdotQdot = Jdot*qdot;

    MatrixXd JWQ = JW * Q;

    VectorXd RHS = -JdotQdot - JWQ - ks * C - kd * Cdot;

    ConjugateGradient<MatrixXd, Eigen::Lower|Eigen::Upper> cg;
    cg.setTolerance(1e-30f);
    cg.setMaxIterations(100);
    cg.compute(JWJT);
    MatrixXd x =  cg.solve(RHS);
    MatrixXd QHat =  J.transpose() * x;


    for (int i = 0; i < particles.size(); i++) {
        //Particle 0's constraint m_Force is stored at index 0 and 1
        particles[i]->m_Force[0] += QHat(i*2, 0);
        particles[i]->m_Force[1] += QHat(i*2+1, 0);
    }
}
