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

    float2 rv = m_body1->m_velocity - m_body0->m_velocity;

    float velAlongNormal = linalg::dot(rv, m_normal);
    if(velAlongNormal > 0.0f)
        return;
    
    float e = std::min(m_body0->m_restitution, m_body1->m_restitution);
    // Determine if we should perform a resting collision or not
    // The idea is if the only thing moving this object is gravity,
    // then the collision should be performed without any restitution
    if( (m_body1->m_force - m_body0->m_force).x < 0.0001f )
        e = 0.0f;

    float j = -(1.0f + e) * velAlongNormal;
    j /= (1.0f / m_body0->m_mass) + (1.0f / m_body1->m_mass);
    
    // Apply impulse
    float2 impulse = m_normal * j;
    
    m_body0->m_velocity -= (1.0f / m_body0->m_mass) * impulse;
    m_body1->m_velocity += (1.0f / m_body1->m_mass) * impulse;
}

void Manifold::PositionalCorrection() const
{
    const float percent = 0.2f; // usually 20% to 80%
    const float slop = 0.01f; // usually 0.01 to 0.1

    const float inv_mass_a = (1.0f / m_body0->m_mass);
    const float inv_mass_b = (1.0f / m_body1->m_mass);

    float2 correction = 
        std::max( m_penetration - slop, 0.0f ) / 
        (inv_mass_a + inv_mass_b) * percent * m_normal;

    m_body0->m_position -= inv_mass_a * correction;
    m_body1->m_position += inv_mass_b * correction;
}