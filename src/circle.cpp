#include "circle.hpp"

#include "manifold.hpp"

#include "GL/freeglut.h"

#include <iostream>

Manifold Circle::accept(std::shared_ptr<ShapeVisitor<Manifold>> visitor)
{
    return visitor->visitCircle(shared_from_this());
}

Manifold Circle::visitAABB(std::shared_ptr<AABB> _shape)
{
    // TODO
    //return Manifold(false);
    return Manifold(
        nullptr,
        nullptr,
        float2(0, 0),
        0.0f,
        false
    );
}

Manifold Circle::visitCircle(std::shared_ptr<Circle> _shape)
{
    bool isHit = true;
    float2 normal = _shape->m_body->GetPosition() - m_body->GetPosition();

    float radius_sum_sqr = m_radius + _shape->m_radius;
    radius_sum_sqr *= radius_sum_sqr;

    // length2 returns length square
    if(linalg::length2(normal) > radius_sum_sqr)
    {
        isHit = false;
    }

    float penetration;
    float distance = linalg::length(normal);
    if(distance != 0)
    {
        penetration = radius_sum_sqr - distance;
        normal = normal / distance;
    }
    else
    {
        penetration = m_radius;
        normal = float2(1, 0);
    }

    // std::cout << "normal = " << normal[0] << " , " << normal[1] << std::endl;
    // std::cout << "penetration = " << penetration << std::endl;

    return Manifold(
        m_body,
        _shape->m_body,
        normal,
        penetration,
        isHit
    );
}

void Circle::Render()
{
    const size_t k_segments = 20;

    glPushMatrix();
    glBegin(GL_LINE_LOOP);
    {
        float theta = 0.0f;
        float inc = M_PI * 2.0f / k_segments;
        for(size_t i = 0; i < k_segments; ++i)
        {
            theta += inc;
            float2 p( std::cos( theta ), std::sin( theta ) );
            p *= m_radius;
            p += m_body->GetPosition();
            glVertex2f( p.x, p.y );
        }
    }
    glEnd( );
    glPopMatrix();
}