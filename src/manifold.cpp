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
    // Ref : https://github.com/RandyGaul/ImpulseEngine/blob/master/Manifold.cpp#L49
    
    // TODO : these values are hard-coded, try to refactor these
    if( linalg::length2(rv) < linalg::length2( 1.0/60.0f * float2(0, -9.8f) ) + 0.0001f )
        e = 0.0f;

    const float inv_mass_a = 
        (m_body0->m_mass != 0.0f) ? (1.0f / m_body0->m_mass) : 0.0f;
    const float inv_mass_b = 
        (m_body1->m_mass != 0.0f) ? (1.0f / m_body1->m_mass) : 0.0f;

    if(inv_mass_a == 0.0f && inv_mass_b == 0.0f)
        return;

    float j = -(1.0f + e) * velAlongNormal;
    j /= inv_mass_a + inv_mass_b;
    
    // Apply impulse
    float2 impulse = m_normal * j;
    
    m_body0->m_velocity -= inv_mass_a * impulse;
    m_body1->m_velocity += inv_mass_b * impulse;

    /**
     *  The following section will be handling frictions.
     */

    // Re-calculate relative velocity after normal impulse is applied.
    float2 rv_after_impulse = m_body1->m_velocity - m_body0->m_velocity;
    // Solve for the tangent vector
    float2 tangent = 
        rv_after_impulse - linalg::dot(rv_after_impulse, m_normal) * m_normal;

    if(linalg::length2(tangent) > 0.0f)
    {
        tangent = linalg::normalize(tangent);
    }

    // Solve for magnitude to apply along the friction vector
    float jt = -1 * linalg::dot( rv_after_impulse, tangent );
    jt /= inv_mass_a + inv_mass_b;

    // Don't apply tiny friction impulses
    if(std::abs(jt) < 0.0001f)
    {
        return;
    }

    // Coulumb's law
    const float sf = std::sqrt(m_body0->m_staticFriction * m_body1->m_staticFriction);
    const float df = std::sqrt(m_body0->m_dynamicFriction * m_body1->m_dynamicFriction);

    float2 tangentImpulse;
    if(std::abs( jt ) < j * sf)
    {
        tangentImpulse = tangent * jt;
    }
    else
    {
        tangentImpulse = tangent * -j * df;
    }

    // Apply friction impulse
    m_body0->m_velocity -= inv_mass_a * tangentImpulse;
    m_body1->m_velocity += inv_mass_b * tangentImpulse;
}

void Manifold::PositionalCorrection() const
{
    const float percent = 0.4f; // usually 20% to 80%
    const float slop = 0.05f; // usually 0.01 to 0.1

    const float inv_mass_a = 
        (m_body0->m_mass != 0.0f) ? (1.0f / m_body0->m_mass) : 0.0f;
    const float inv_mass_b = 
        (m_body1->m_mass != 0.0f) ? (1.0f / m_body1->m_mass) : 0.0f;

    if(inv_mass_a == 0.0f && inv_mass_b == 0.0f)
        return;

    float2 correction = 
        (std::max( m_penetration - slop, 0.0f ) / (inv_mass_a + inv_mass_b))
        * percent * m_normal;

    m_body0->m_position -= inv_mass_a * correction;
    m_body1->m_position += inv_mass_b * correction;
}