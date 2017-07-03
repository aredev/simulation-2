#pragma once

#include <gfx/vec2.h>

class Particle {
public:

    Particle(const Vec2f &ConstructPos, float m);

    virtual ~Particle();

    void reset();

    virtual void draw();

    int row; //Used to know which row to place the particle in the J matrix
    Vec2f m_ConstructPos;
    Vec2f m_Position;
    Vec2f m_Velocity;
    Vec2f m_Force;
    float m_Mass;
    float m_Colour;

    void drawForce();

    void drawVelocity();

    void setColour(float c);
};
