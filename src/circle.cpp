#include "circle.hpp"

#include "manifold.hpp"
#include "aabb.hpp"
#include "collision.hpp"

#include "GL/freeglut.h"

#include <cmath>

Manifold Circle::accept(std::shared_ptr<ShapeVisitor<Manifold>> visitor)
{
    return visitor->visitCircle(shared_from_this());
}

Manifold Circle::visitAABB(std::shared_ptr<AABB> _shape)
{
    // in impulse engine, the normal is flipped ( * -1 )
    // because in that architecture, the body0 and body1 is already
    // decided before manifold is actually generated 
    // ( the normal and penetration ), so it has to be reversed
    // in order to match the API. In this project, since the field
    // of body0 and body1 is filled in 'when' computing manifold,
    // we do not need to do that (reversing normal).

    auto manifold = CollisionHelper::GenerateManifold(
        _shape,
        shared_from_this()
    );

    return manifold;
}

Manifold Circle::visitCircle(std::shared_ptr<Circle> _shape)
{
    bool isHit = true;
    float2 normal = _shape->m_body->GetPosition() - m_body->GetPosition();

    float radius_sum = m_radius + _shape->m_radius;
    float radius_sum_sqr = radius_sum * radius_sum;

    // length2 returns length square
    if(linalg::length2(normal) > radius_sum_sqr)
    {
        isHit = false;
    }

    float penetration;
    float distance = linalg::length(normal);
    if(distance != 0)
    {
        penetration = radius_sum - distance;
        normal = normal / distance;
    }
    else
    {
        penetration = m_radius;
        normal = float2(1, 0);
    }

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