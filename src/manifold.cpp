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

    // std::cout << "m_body0->m_velocity = " << m_body0->m_velocity[0] << 
    //     " , " << m_body0->m_velocity[1] << std::endl;
   
    // std::cout << "m_body1->m_velocity = " << m_body1->m_velocity[0] << 
    //     " , " << m_body1->m_velocity[1] << std::endl;

    float2 rv = m_body1->m_velocity - m_body0->m_velocity;

    float velAlongNormal = linalg::dot(rv, m_normal);
    if(velAlongNormal > 0.0f)
        return;
    
    float e = std::min(m_body0->m_restitution, m_body1->m_restitution);
    float j = -(1.0f + e) * velAlongNormal;
    j /= (1.0f / m_body0->m_mass) + (1.0f / m_body1->m_mass);
    
    // Apply impulse
    float2 impulse = m_normal * j;
    
    m_body0->m_velocity -= 1.0f / m_body0->m_mass * impulse;
    m_body1->m_velocity += 1.0f / m_body1->m_mass * impulse;
}