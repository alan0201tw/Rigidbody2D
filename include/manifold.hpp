#pragma once

#include "rigidbody2D.hpp"

class Manifold
{
    typedef linalg::aliases::float2 float2;
public:
    std::shared_ptr<RigidBody2D> m_body0, m_body1;
    int m_contactPointCount;
    std::array<float2, 2> m_contactPoints;
    float2 m_normal;
    float m_penetration;

    bool m_isHit;

public:

    Manifold(
        std::shared_ptr<RigidBody2D> _body0, 
        std::shared_ptr<RigidBody2D> _body1,
        int _contactPointCount,
        std::array<float2, 2> _contactPoints,
        float2 _normal,
        float _penetration,
        bool _isHit);

    void Resolve() const;
    void PositionalCorrection() const;
};