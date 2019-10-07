#include "manifold.hpp"

void Manifold::Resolve() const
{
    // float2 rv = m_body0->m_velocity - m_body1->m_velocity;

    // float velAlongNormal = linalg::dot(rv, m_normal);
    // if(velAlongNormal > 0.0f)
    //     return;
    
    // float e = std::min(m_body0->m_restitution, m_body1->m_restitution);
    // float j = -(1.0f + e) * velAlongNormal;
    // j /= (1.0f / m_body0->m_mass) + (1.0f / m_body1->m_mass);
    
    // // Apply impulse
    // float2 impulse = m_normal * j;
    
    // m_body0->m_velocity -= 1.0f / m_body0->m_mass * impulse;
    // m_body1->m_velocity += 1.0f / m_body1->m_mass * impulse;
}