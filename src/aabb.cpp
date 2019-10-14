#include "aabb.hpp"

#include "manifold.hpp"
#include "circle.hpp"
#include "collision.hpp"

#include "GL/freeglut.h"

Manifold AABB::accept(std::shared_ptr<ShapeVisitor<Manifold>> visitor)
{
    return visitor->visitAABB(shared_from_this());
}

Manifold AABB::visitAABB(std::shared_ptr<AABB> _shape)
{
    bool isHit = false;
    float2 normal = _shape->m_body->GetPosition() - m_body->GetPosition();

    float a_extent_x = m_extent.x / 2.0f;
    float b_extent_x = _shape->m_extent.x / 2.0f;

    float x_overlap = a_extent_x + b_extent_x - std::abs(normal.x);

    float penetration = 0.0f;
    if(x_overlap > 0.0f)
    {
        // Calculate half extents along y axis for each object
        float a_extent_y = m_extent.y / 2.0f;
        float b_extent_y = _shape->m_extent.y / 2.0f;

        float y_overlap = a_extent_y + b_extent_y - std::abs(normal.y);

        if(y_overlap > 0.0f)
        {
            // Find out which axis is axis of least penetration
            if(x_overlap > y_overlap)
            {
                normal = (normal.y < 0.0f) ? float2(0, -1) : float2(0, 1);
                penetration = y_overlap;
                isHit = true;
            }
            else
            {
                normal = (normal.x < 0.0f) ? float2(-1, 0) : float2(1, 0);
                penetration = x_overlap;
                isHit = true;
            }
        }
    }
    
    // No separating axis found, therefor there is at least one overlapping axis
    return Manifold(
        m_body,
        _shape->m_body,
        linalg::normalize(normal),
        penetration,
        isHit
    );
}

Manifold AABB::visitCircle(std::shared_ptr<Circle> _shape)
{
    auto manifold = CollisionHelper::GenerateManifold(
        shared_from_this(),
        _shape
    );

    return manifold;
}

void AABB::Render()
{
    glPushMatrix();

    glTranslatef(m_body->GetPosition().x, m_body->GetPosition().y, 0);
    glRotatef(m_body->m_orientation, 0, 0, 1);

    glBegin(GL_LINE_LOOP);
    {
        float2 half_extent = m_extent / 2.0f;

        glVertex2f(0 - half_extent[0], 0 - half_extent[1]);
        glVertex2f(0 - half_extent[0], 0 + half_extent[1]);
        glVertex2f(0 + half_extent[0], 0 + half_extent[1]);
        glVertex2f(0 + half_extent[0], 0 - half_extent[1]);
    }
    glEnd();

    // glPointSize( std::min(m_extent.x, m_extent.y) * 1.0f );

    glBegin(GL_POINTS);
    {
        glPushMatrix();

        glVertex2f(0, 0);

        glPopMatrix();
    }
    glEnd();

    glPopMatrix();
}