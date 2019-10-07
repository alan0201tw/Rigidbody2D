#pragma once

#include "rigidbody2D.hpp"

class Manifold
{
    typedef linalg::aliases::float2 float2;
private:
    std::shared_ptr<RigidBody2D> m_body0, m_body1;
    float2 m_normal;
    float m_penetration;
    bool m_isHit;

public:

    Manifold(bool _isHit) : m_isHit(_isHit) {}

    void Resolve() const;

// TODO
};