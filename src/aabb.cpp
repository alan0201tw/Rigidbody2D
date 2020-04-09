#include "aabb.hpp"

#include "manifold.hpp"
#include "circle.hpp"
#include "collision.hpp"
#include "util.hpp"

#include <vector>
#include <algorithm>
#include <iostream>

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
    // rotate B to make A an AABB, check if any vertices of B is inside A
    // then, rotate A to make A an AABB, check if any vertices of A is inside B

    // TODO : check if any manifolds have "roughly" same normal
    // if yes, they should be forming a face that collides
    std::array<float2, 2> contactPoints;

    Manifold manifold0 = isVertexOfBInsideA(shared_from_this(), _shape);
    Manifold manifold1 = isVertexOfBInsideA(_shape, shared_from_this());
    if(manifold0.m_isHit == true && manifold1.m_isHit == true)
    {
        for(int i = 0; i < manifold1.m_contactPointCount; ++i)
        {
            manifold0.m_contactPoints[manifold0.m_contactPointCount] = 
                manifold1.m_contactPoints[i];
            ++manifold0.m_contactPointCount;

            // if(manifold0.m_contactPointCount > 1)
            //     std::cout << "manifold0.m_contactPointCount = " << manifold0.m_contactPointCount << "\n";
        }
        if(manifold0.m_contactPointCount != 0)
            manifold0.m_penetration /= manifold0.m_contactPointCount;

        return manifold0;
    }
    else if(manifold0.m_isHit == true && manifold1.m_isHit == false)
    {
        return manifold0;
    }
    else
    {
        return manifold1;
    }
}

Manifold AABB::isVertexOfBInsideA(std::shared_ptr<const AABB> _a, std::shared_ptr<const AABB> _b)
{
    float rotationDifference = 
        _b->m_body->GetOrientation() - _a->m_body->GetOrientation();

    float2x2 rotationMatrix = 
        getRotationMatrix(_a->m_body->GetOrientation());
    float2x2 rotationMatrixDiff = 
        getRotationMatrix(rotationDifference);
    
    float2 rotatedBPosition = 
        _a->m_body->GetPosition()
        + linalg::mul(linalg::transpose(rotationMatrix)
			, (_b->m_body->GetPosition() - _a->m_body->GetPosition()));

    const float b_extent_x = _b->m_extent.x / 2.0f;
    const float b_extent_y = _b->m_extent.y / 2.0f;
    const float a_extent_x = _a->m_extent.x / 2.0f;
    const float a_extent_y = _a->m_extent.y / 2.0f;

    std::array<float2, 4> verticesOfB = 
    {
        rotatedBPosition + linalg::mul(rotationMatrixDiff, float2( b_extent_x,  b_extent_y)),
        rotatedBPosition + linalg::mul(rotationMatrixDiff, float2( b_extent_x, -b_extent_y)),
        rotatedBPosition + linalg::mul(rotationMatrixDiff, float2(-b_extent_x,  b_extent_y)),
        rotatedBPosition + linalg::mul(rotationMatrixDiff, float2(-b_extent_x, -b_extent_y))
    };

    Manifold manifold = Manifold(
            _a->m_body,
            _b->m_body,
            0,
            { },
            float2(0, 0),
            0.0f,
            false
        );

    for(int i = 0; i < 4; ++i)
    {
        const float2 vertexPos = verticesOfB[i];

        const float2 maxPosA = _a->m_body->GetPosition() + float2(a_extent_x, a_extent_y);
        const float2 minPosA = _a->m_body->GetPosition() - float2(a_extent_x, a_extent_y);

        if(vertexPos.x < maxPosA.x && vertexPos.y < maxPosA.y
        && vertexPos.x > minPosA.x && vertexPos.y > minPosA.y)
        {
            float distanceToBoundaryX = std::min( maxPosA.x - vertexPos.x , vertexPos.x - minPosA.x );
            float distanceToBoundaryY = std::min( maxPosA.y - vertexPos.y , vertexPos.y - minPosA.y );
            
            float penetration = -1.0f;
            float2 delta = vertexPos - _a->m_body->GetPosition();
            float2 worldNormal;

            if( distanceToBoundaryY > distanceToBoundaryX )
            {
                delta = (delta.x < 0.0f) ? float2(-1, 0) : float2(1, 0);
                penetration = std::min( maxPosA.x - vertexPos.x , vertexPos.x - minPosA.x );
            }
            else
            {
                delta = (delta.y < 0.0f) ? float2(0, -1) : float2(0, 1);
                penetration = std::min( maxPosA.y - vertexPos.y , vertexPos.y - minPosA.y );
            }

            worldNormal = safe_normalize(linalg::mul(rotationMatrix, delta));

            float2 contactPoint = 
                _a->m_body->GetPosition()
                + linalg::mul(rotationMatrix, (vertexPos - _a->m_body->GetPosition()));

            if(manifold.m_isHit == false)
            {
                manifold = Manifold(
                    _a->m_body,
                    _b->m_body,
                    1,
                    { contactPoint },
                    worldNormal,
                    penetration,
                    true
                );
            }
            else if(worldNormal == manifold.m_normal)
            {
                // std::cout << "manifold normal = " 
                //     << manifold.m_normal.x << ", " << manifold.m_normal.y << "\n";
                // std::cout << "normal = " 
                //     << worldNormal.x << ", " << worldNormal.y << "\n";

                manifold.m_contactPoints[manifold.m_contactPointCount] = contactPoint;
                ++manifold.m_contactPointCount;
            }
        }
    }
    if(manifold.m_contactPointCount != 0)
        manifold.m_penetration /= manifold.m_contactPointCount;

    return manifold;
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