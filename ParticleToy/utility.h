//
// Created by Tomirio on 20-6-2017.
//

#ifndef SIMULATION_1_UTILITY_H
#define SIMULATION_1_UTILITY_H

#endif //SIMULATION_1_UTILITY_H

#include <Eigen/StdVector>
#include "particles/Particle.h"

namespace utility {

    float normalize(float value, float min, float max);

    float euclideanDistance(Vec2f a, Vec2f b);

    bool isWithinProximity(Vec2f a, Vec2f b, float threshold);

    Vec2f getTransformedCoordinates(int x, int y);

    bool noCloseParticles(std::vector<Particle *> particles, Vec2f v, float threshold);
}