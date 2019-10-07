#include "circle.hpp"

#include "manifold.hpp"

#include "GL/freeglut.h"

Manifold Circle::accept(std::shared_ptr<ShapeVisitor<Manifold>> visitor)
{
    return visitor->visitCircle(shared_from_this());
}

Manifold Circle::visitAABB(std::shared_ptr<AABB> _shape)
{
    // TODO
    return Manifold(false);
}

Manifold Circle::visitCircle(std::shared_ptr<Circle> _shape)
{
    float2 position = m_body->GetPosition();
    float2 otherPosition = _shape->m_body->GetPosition();

    float radius_sum_sqr = m_radius + _shape->m_radius;
    radius_sum_sqr *= radius_sum_sqr;

    float x_sum = (position[0] + otherPosition[0]);
    float x_sum_sqr = x_sum * x_sum;

    float y_sum = (position[1] + otherPosition[1]);
    float y_sum_sqr = y_sum * y_sum;

    bool isColliding = (radius_sum_sqr < x_sum_sqr + y_sum_sqr);
    return Manifold(isColliding);
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