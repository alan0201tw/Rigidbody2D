#include "collision.hpp"

#include <algorithm>
#include <iostream>

#include "linalg.h"

Manifold CollisionHelper::GenerateManifold(std::shared_ptr<const AABB> _a, std::shared_ptr<const Circle> _b)
{
    auto normal = _b->m_body->GetPosition() - _a->m_body->GetPosition();
    auto closest = normal;

    float x_half_extent = _a->m_extent.x / 2.0f;
    float y_half_extent = _a->m_extent.y / 2.0f;

    closest.x = std::clamp( closest.x, -x_half_extent, x_half_extent );
    closest.y = std::clamp( closest.y, -y_half_extent, y_half_extent );

    bool inside = false;

    if( normal == closest )
    {
        inside = true;

        if(std::abs(normal.x) > std::abs(normal.y))
        {
            if(closest.x > 0)
                closest.x = x_half_extent;
            else
                closest.x = -x_half_extent;
        }
        else
        {
            if(closest.y > 0)
                closest.y = y_half_extent;
            else
                closest.y = -y_half_extent;
        }
    }
    
    normal = normal - closest;
    float d = linalg::length2(normal);
    float r = _b->m_radius;

    bool isHit = true;
    if( d > r * r && inside == false )
    {
        isHit = false;
    }

    // if inside, d < r
    normal = linalg::normalize(normal);

    d = std::sqrt(d);

    return Manifold(
        _a->m_body,
        _b->m_body,
        (inside == true) ? -normal : normal,
        (inside == true) ? r + d : r - d,
        isHit
    );
}