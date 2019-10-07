#pragma once

#include "linalg.h"

#include <memory>

class Shape;

class RigidBody2D
{
    typedef linalg::aliases::float2 float2;
private:
    float2 m_position;
    float2 m_velocity;
    float2 m_force;

    float m_restitution;
    float m_mass;

    std::shared_ptr<Shape> m_shape;

public:
    inline std::shared_ptr<Shape> GetShape() { return m_shape; }

    inline float2 GetPosition() { return m_position; }

    friend class Manifold;
};