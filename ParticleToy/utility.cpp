//
// Created by Tomirio on 20-6-2017.
//

#include <math.h>
#include "particles/Particle.h"
#include "GL/freeglut.h"
#include <vector>

using namespace std;

namespace utility {

/**
 * See https://gamedev.stackexchange.com/questions/32555/how-do-i-convert-between-two-different-2d-coordinate-systems
 * @param value The value to normalize
 * @param min The min value
 * @param max The max value
 * @return normalized value
 */
    float normalize(float value, float min, float max) {
        return fabsf((value - min) / (max - min));
    }

    /**
     * Calculate the Euclidean Distance between two vectors
     * @param a vector
     * @param b vector
     * @return Distance between vector a and b
     */
    float euclideanDistance(Vec2f a, Vec2f b) {
        return sqrtf(powf(a[0] - b[0], 2) + powf(a[1] - b[1], 2));
    }

    /**
     * Determine whether two vectors are at most threshold distance apart.
     * @param a First vector
     * @param b Second vector
     * @param threshold The threshold
     * @return <code>True</code> if dist(a,b) <= threshold, <code>False</code> otherwise.
     */
    bool isWithinProximity(Vec2f a, Vec2f b, float threshold) {
        float dist = euclideanDistance(a, b);
        return dist <= threshold;
    }


    /**
     * Transforms x and y coordinates retrieved from a mouse click to the coordinate system used to draw points on the
     * screen.
     * @param x The x value of the mouse
     * @param y  The y value of the mouse
     * @return  Vec2f of transformed coordinates
     */
    Vec2f getTransformedCoordinates(int x, int y) {
        float max = 1.0f;
        float min = -1.0f;
        float windowWidth = glutGet(GLUT_WINDOW_WIDTH);
        float windowHeight = glutGet(GLUT_WINDOW_HEIGHT);
        // Normalize the x and y coordinates that indicate the mouse position
        float xPercent = normalize(x, 0, windowWidth);
        float yPercent = normalize(y, 0, windowHeight);
        // Compute values in the new coordinate system
        float destX = xPercent * fabsf(max - min) + min;
        float destY = yPercent * fabsf(max - min) + min;
        destY *= -1;
        return Vec2f(destX, destY);
    }

    /**
     * Checks whether a given vector is not within a threshold of other particles
     * @param particles
     * @param p
     * @param threshold
     * @return
     */
    bool noCloseParticles(std::vector<Particle *> particles, Vec2f v, float threshold) {
        for (auto &particle: particles) {
            float px = particle->m_Position[0];
            float py = particle->m_Position[1];
            if (isWithinProximity(v, particle->m_Position, threshold)) {
                return false;
            }
        }
        return true;
    }

}

