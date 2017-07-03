#include "SetUp.h"
#include "../forces/SpringForce.h"
#include "../forces/GravityForce.h"
#include "../constraints/RodConstraint.h"
#include "../constraints/CircularWireConstraint.h"
#include "../constraints/SlidingWireConstraint.h"
#include "../forces/HorizontalForce.h"
#include "../forces/WallForce.h"
#include "../forces/AngularSpringForce.h"

double particleMass = 1.0f;
Vec2f center = Vec2f(0.0, 0.0);

void SetUp::setUpGravity(std::vector<Particle *> &pVector, std::vector<Force *> &forces) {
    Particle *p1 = new Particle(center, particleMass);
    Particle *p2 = new Particle(Vec2f(0.2, 0.0), particleMass);

    pVector.push_back(p1);
    pVector.push_back(p2);
    Force *gravityOne = new GravityForce(pVector);
    forces.push_back(gravityOne);
}

void SetUp::setUpSpringforce(std::vector<Particle *> &pVector, std::vector<Force *> &forces) {
    double restLength = 0.5;
    Particle *p1 = new Particle(Vec2f(0.0, 0.0), particleMass);
    Particle *p2 = new Particle(Vec2f(0.0, -0.2), particleMass);

    pVector.push_back(p1);
    pVector.push_back(p2);

    Force *springForce = new SpringForce(p1, p2, restLength, 0.1, 0.1);

    forces.push_back(springForce);
}

void SetUp::setUpRodConstraint(std::vector<Particle *> &pVector, std::vector<Force *> &forces,
                               std::vector<ConstraintForce *> &constraints) {
    Particle *centerParticle = new Particle(center, particleMass);
    Particle *bottomParticle = new Particle(Vec2f(0.0, -0.2), particleMass);

    pVector.push_back(centerParticle);
    pVector.push_back(bottomParticle);

    ConstraintForce *rodConstraintForce = new RodConstraint(bottomParticle, centerParticle, 0.2);
    constraints.push_back(rodConstraintForce);

    std::vector<Particle *> gravityParticles;
    gravityParticles.push_back(bottomParticle);
    Force *gravityForce = new GravityForce(gravityParticles);
    forces.push_back(gravityForce);
}

void SetUp::setUpCircularConstraint(std::vector<Particle *> &pVector, std::vector<Force *> &forces,
                                    std::vector<ConstraintForce *> &constraints) {
    Particle *rightParticle = new Particle(Vec2f(0.2, 0.0), particleMass);
    pVector.push_back(rightParticle);

    std::vector<Particle *> gravityParticles;
    gravityParticles.push_back(rightParticle);
    Force *gravity = new GravityForce(gravityParticles);
    forces.push_back(gravity);

    ConstraintForce *circularWire = new CircularWireConstraint(rightParticle, center, 0.2f);
    constraints.push_back(circularWire);
}

void SetUp::setUpHorizontalConstraint(std::vector<Particle *> &pVector, std::vector<Force *> &forces,
                                      std::vector<ConstraintForce *> &constraints) {
    Particle *lineParticle = new Particle(center, particleMass);
    pVector.push_back(lineParticle);

    ConstraintForce *slidingWire = new SlidingWireConstraint(lineParticle, 0.4);
    constraints.push_back(slidingWire);

    Force *horizontalForce = new HorizontalForce(lineParticle, 0.01f);
    Force *gravityForce = new GravityForce(pVector);
    forces.push_back(horizontalForce);
    forces.push_back(gravityForce);
}

void SetUp::setUpMixedConstraint(std::vector<Particle *> &pVector, std::vector<Force *> &forces,
                                 std::vector<ConstraintForce *> &constraints) {
    Particle *lineParticle = new Particle(Vec2f(0.1, 0.4), particleMass);
    Particle *circleParticle = new Particle(Vec2f(0.0, 0.2), particleMass);
    pVector.push_back(lineParticle);
    pVector.push_back(circleParticle);

    ConstraintForce *slidingWire = new SlidingWireConstraint(lineParticle, 0.4);
    constraints.push_back(slidingWire);

    ConstraintForce *circularWire = new CircularWireConstraint(circleParticle, center, 0.2f);
    constraints.push_back(circularWire);

    ConstraintForce *rodConstraint = new RodConstraint(lineParticle, circleParticle, 0.5f);
    constraints.push_back(rodConstraint);

    std::vector<Particle *> gravity;
    gravity.push_back(circleParticle);
    Force *gravityForce = new GravityForce(gravity);
    forces.push_back(gravityForce);
}

void SetUp::setUpSlidingCloth(std::vector<Particle *> &pVector, std::vector<Force *> &forces,
                              std::vector<ConstraintForce *> &constraints, int N) {
    Vec2f center = Vec2f(0, 0);
    double spacing = 0.2;

    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            Particle *p = new Particle(Vec2f(center[0] + j * spacing, center[1] - i * spacing), 1.0f);
            pVector.push_back(p);
            if (i == 0) {
                constraints.push_back(new SlidingWireConstraint(p, 0.4));
            }

            if (j > 0) {
                forces.push_back(new SpringForce(pVector[N * i + j], pVector[N * i + j - 1], spacing, 1, 1));
            }

            if (i > 0) {
                forces.push_back(new SpringForce(pVector[N * i + j], pVector[N * i + j - N], spacing, 1, 1));
            }
        }
    }

}

void SetUp::setUpSlidingClothWall(std::vector<Particle *> &pVector, std::vector<Force *> &forces,
                                  std::vector<ConstraintForce *> &constraints, int size) {
    setUpSlidingCloth(pVector, forces, constraints, size);

    for (int i = 0; i < size * size; ++i) {
        //Set the wall after all the constraints (at the right side) and add an additional half offset between the
        //last particle column and the wall
        Force *wall = new WallForce(pVector[i], size * 0.2 + 0.1f);
        forces.push_back(wall);
    }
}

void SetUp::setUpAngularSpring(std::vector<Particle *> &pVector, std::vector<Force *> &forces) {
    const double dist = 0.2;
    Vec2f center = Vec2f(0, -0.5);
    Vec2f left = Vec2f(-dist, 0);
    Vec2f right = Vec2f(dist, 0);
    Particle *p1 = new Particle(left, 1);
    Particle *p2 = new Particle(center, 1);
    Particle *p3 = new Particle(right, 1);
    Force *angularSpringForce = new AngularSpringForce(p1, p2, p3, 0.2, 50, 0.2, 0.1);
//    Force *sp1 = new SpringForce(p1, p2, 0.2, 0.2, 0.2);
//    Force *sp2 = new SpringForce(p2, p3, 0.2, 0.2, 0.2);
    pVector.push_back(p1);
    pVector.push_back(p2);
    pVector.push_back(p3);
    forces.push_back(angularSpringForce);
//    forces.push_back(sp1);
//    forces.push_back(sp2);
    return;
}
