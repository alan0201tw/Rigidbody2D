#include "aabb.hpp"

#include "manifold.hpp"
#include "circle.hpp"
#include "collision.hpp"
#include "util.hpp"

#include "GL/freeglut.h"

AABB::float2 AABB::getSupportPoint(const float2& dir) const
{
    // init as max
    float bestProjection = -1e9f;
    float2 bestVertex;

    const float2 half_extent = m_extent / 2.0f;
    std::array<float2, 4> vertices = 
    {
        - half_extent,
          half_extent,
        float2(half_extent.x, -half_extent.y),
        float2(-half_extent.x, half_extent.y),
    };

    for(size_t i = 0; i < 4; i++)
    {
        float2 v = vertices[i];
        float projection = linalg::dot( v, dir );
    
        if(projection > bestProjection)
        {
            bestVertex = v;
            bestProjection = projection;
        }
    }

    return bestVertex;
}

Manifold AABB::accept(std::shared_ptr<const ShapeVisitor<Manifold>> visitor) const
{
    return visitor->visitAABB(shared_from_this());
}

Manifold AABB::visitAABB(std::shared_ptr<const AABB> _shape) const
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
    normal = linalg::normalize(normal);
    float2 contactPoint = m_body->GetPosition() + penetration * normal;
    
    // No separating axis found, therefor there is at least one overlapping axis
    return Manifold(
        m_body,
        _shape->m_body,
        contactPoint,
        linalg::normalize(normal),
        penetration,
        isHit
    );
}

Manifold AABB::visitCircle(std::shared_ptr<const Circle> _shape) const
{
    auto manifold = CollisionHelper::GenerateManifold(
        shared_from_this(),
        _shape
    );

    return manifold;
}

void AABB::Render() const
{
    glPushMatrix();

    glTranslatef(m_body->GetPosition().x, m_body->GetPosition().y, 0);
    glRotatef(radianToDegree(m_body->GetOrientation()), 0, 0, 1);

    glBegin(GL_LINE_LOOP);
    {
        float2 half_extent = m_extent / 2.0f;

        glVertex2f(0 - half_extent[0], 0 - half_extent[1]);
        glVertex2f(0 - half_extent[0], 0 + half_extent[1]);
        glVertex2f(0 + half_extent[0], 0 + half_extent[1]);
        glVertex2f(0 + half_extent[0], 0 - half_extent[1]);
    }
    glEnd();

    glBegin(GL_POINTS);
    {
        glPushMatrix();

        glVertex2f(0, 0);

        glPopMatrix();
    }
    glEnd();

    glPopMatrix();
}