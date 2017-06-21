#ifndef SIMULATION_1_ANGULARSPRINGFORCE_H
#define SIMULATION_1_ANGULARSPRINGFORCE_H


#include "Force.h"

class AngularSpringForce : public Force {


public:
    AngularSpringForce();

    AngularSpringForce(Particle *p1, Particle *p2, Particle *p3, double r, double ra, double ks, double kd);

    void computeForce() override;

    void draw() override;

private:

    /**
     * Stiffness constant
     */
    double m_ks;
    /**
     * Damping constant
     */
    double m_kd;
    /**
     * Rest angle
     */
    double m_ra;

    /**
     * Rest length
     */
    double m_r;
    /**
     * Particles
     */
    Particle * m_p1;
    Particle * m_p2;
    Particle * m_p3;
};




#endif //SIMULATION_1_ANGULARSPRINGFORCE_H
