#include "manifold.hpp"

#include <iostream>

Manifold::Manifold(
    std::shared_ptr<RigidBody2D> _body0, 
    std::shared_ptr<RigidBody2D> _body1,
    float2 _normal,
    float _penetration,
    bool _isHit)
    : m_body0(_body0), m_body1(_body1), m_normal(_normal),
      m_penetration(_penetration), m_isHit(_isHit)
    {}

void Manifold::Resolve() const
{
    if(m_isHit == false)
        return;

    // if(m_body0->m_mass == 0.0f && m_body1->m_mass == 0.0f)
    // {
    //     m_body0->m_velocity = float2(0.0f, 0.0f);
    //     m_body1->m_velocity = float2(0.0f, 0.0f);
    // }

    float2 rv = m_body1->m_velocity - m_body0->m_velocity;

    float velAlongNormal = linalg::dot(rv, m_normal);
    if(velAlongNormal > 0.0f)
        return;
    
    float e = std::min(m_body0->m_restitution, m_body1->m_restitution);
    // Determine if we should perform a resting collision or not
    // The idea is if the only thing moving this object is gravity,
    // then the collision should be performed without any restitution
    // Ref : https://github.com/RandyGaul/ImpulseEngine/blob/master/Manifold.cpp#L49
    
    // TODO : these values are hard-coded, try to refactor these
    if( linalg::length2(rv) < linalg::length2( 1.0/60.0f * float2(0, -9.8f) ) + 0.0001f )
        e = 0.0f;

    const float inv_mass_a = 
        (m_body0->m_mass != 0.0f) ? (1.0f / m_body0->m_mass) : 0.0f;
    const float inv_mass_b = 
        (m_body1->m_mass != 0.0f) ? (1.0f / m_body1->m_mass) : 0.0f;

    float j = -(1.0f + e) * velAlongNormal;
    j /= inv_mass_a + inv_mass_b;
    
    // Apply impulse
    float2 impulse = m_normal * j;
    
    m_body0->m_velocity -= inv_mass_a * impulse;
    m_body1->m_velocity += inv_mass_b * impulse;
}

void Manifold::PositionalCorrection() const
{
    const float percent = 0.4f; // usually 20% to 80%
    const float slop = 0.05f; // usually 0.01 to 0.1

    const float inv_mass_a = 
        (m_body0->m_mass != 0.0f) ? (1.0f / m_body0->m_mass) : 0.0f;
    const float inv_mass_b = 
        (m_body1->m_mass != 0.0f) ? (1.0f / m_body1->m_mass) : 0.0f;

    float2 correction = 
        (std::max( m_penetration - slop, 0.0f ) / (inv_mass_a + inv_mass_b))
        * percent * m_normal;

    m_body0->m_position -= inv_mass_a * correction;
    m_body1->m_position += inv_mass_b * correction;
}