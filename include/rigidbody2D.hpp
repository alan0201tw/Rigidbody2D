#pragma once

#include "linalg.h"

#include <memory>

class Shape;

class RigidBody2D
{
    typedef linalg::aliases::float2 float2;
private:
    float2 m_position;

public:
    float2 m_velocity;
    float2 m_force;

    float m_restitution;
    float m_mass;

    float m_staticFriction;
    float m_dynamicFriction;

    // Angular components
    float m_orientation; // radians
    float m_angularVelocity;
    float m_torque;
    float m_inertia; // moment of inertia

    std::shared_ptr<Shape> m_shape;

public:
    RigidBody2D(
        std::shared_ptr<Shape> _shape, float2 _position, float _restitution, 
        float _mass, float _staticFriction, float _dynamicFriction)
        : m_position(_position), m_velocity(float2(0, 0)), m_force(float2(0, 0))
        , m_restitution(_restitution), m_mass(_mass)
        , m_staticFriction(_staticFriction), m_dynamicFriction(_dynamicFriction)
        , m_orientation(0.0f), m_angularVelocity(0.0f), m_torque(0.0f), m_inertia(1.0f)
        , m_shape(_shape) {}

    inline std::shared_ptr<Shape> GetShape() { return m_shape; }

    inline float2 GetPosition() { return m_position; }

    friend class Manifold;
    friend class ExplicitEulerIntegrator;
};