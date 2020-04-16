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

static bool isCloseEnough(float2 dir0, float2 dir1)
{
	return linalg::angle(dir0, dir1) < 0.1f;
}

Manifold AABB::visitAABB(std::shared_ptr<const AABB> _shape) const
{
	// rotate B to make A an AABB, check if any vertices of B is inside A
	// then, rotate A to make A an AABB, check if any vertices of A is inside B

	// TODO : check if any manifolds have "roughly" same normal
	// if yes, they should be forming a face that collides
	std::vector<Manifold> manifolds;
	{
		std::vector<Manifold> manifolds0 = isVertexOfBInsideA(shared_from_this(), _shape);
		std::vector<Manifold> manifolds1 = isVertexOfBInsideA(_shape, shared_from_this());

        for (size_t idx = 0; idx < manifolds1.size(); ++idx)
        {
            std::swap(manifolds1[idx].m_body0, manifolds1[idx].m_body1);
            manifolds1[idx].m_normal *= -1.0f;
        }

		manifolds.reserve(manifolds0.size() + manifolds1.size()); // preallocate memory
		manifolds.insert(manifolds.end(), manifolds0.begin(), manifolds0.end());
		manifolds.insert(manifolds.end(), manifolds1.begin(), manifolds1.end());
	}
	// find if any manifold shares the same ( or similar ) normal vector
	float leastPenetration = 1e9f;
	Manifold leastPenetrationManifold = Manifold(
		m_body, _shape->m_body, 0, {}, float2(0, 0), 0.0f, false);

	for (size_t idx = 0; idx < manifolds.size(); ++idx)
	{
		if (manifolds[idx].m_penetration < leastPenetration)
		{
			leastPenetrationManifold = manifolds[idx];
			leastPenetration = manifolds[idx].m_penetration;
		}
        
		for (size_t oidx = idx + 1; oidx < manifolds.size(); ++oidx)
		{
			if (isCloseEnough(manifolds[idx].m_normal, manifolds[oidx].m_normal))
			{
				manifolds[idx].m_contactPoints[manifolds[idx].m_contactPointCount] =
					manifolds[oidx].m_contactPoints[0];

				++manifolds[idx].m_contactPointCount;
				// manifolds[idx].m_penetration /= manifolds[idx].m_contactPointCount;
				return manifolds[idx];
			}
		}
	}

	return leastPenetrationManifold;
}

std::vector<Manifold> AABB::isVertexOfBInsideA(std::shared_ptr<const AABB> _a, std::shared_ptr<const AABB> _b)
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

	std::vector<Manifold> manifolds;

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
                penetration = (delta.x < 0.0f) ? vertexPos.x - minPosA.x : maxPosA.x - vertexPos.x;
            }
            else
            {
                delta = (delta.y < 0.0f) ? float2(0, -1) : float2(0, 1);
                penetration = (delta.y < 0.0f) ? vertexPos.y - minPosA.y : maxPosA.y - vertexPos.y;
            }

            worldNormal = safe_normalize(linalg::mul(rotationMatrix, delta));

            float2 contactPoint = 
                _a->m_body->GetPosition()
                + linalg::mul(rotationMatrix, (vertexPos - _a->m_body->GetPosition()));

			manifolds.emplace_back(
                _a->m_body,
                _b->m_body,
                1,
                std::array<float2, 2>{ contactPoint },
                worldNormal,
                penetration,
                true
            );
        }
    }

    return manifolds;
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