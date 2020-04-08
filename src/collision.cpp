#include "collision.hpp"

#include "util.hpp"

#include <algorithm>
#include <iostream>

#include "linalg.h"

Manifold CollisionHelper::GenerateManifold(std::shared_ptr<const AABB> _a, std::shared_ptr<const Circle> _b)
{
	// std::cout << "rotMat of 30 deg\n";
	// float rad = 30.0f / 180.0f * std::acos(-1.0f);
	// float2x2 rm = getRotationMatrix(rad);
	// std::cout << rm[0][0] << ", ";
	// std::cout << rm[1][0] << ", \n";
	// std::cout << rm[0][1] << ", ";
	// std::cout << rm[1][1] << ", \n";

	// float2 rotVec = linalg::mul(rm, float2(1, 0));
	// std::cout << rotVec.x << ", " << rotVec.y << "\n\n";

	// do inverse rotation to treat the OBB as AABB
	float2x2 rotationMatrix = getRotationMatrix(_a->m_body->GetOrientation());
	
	float2 rotatedCircleCenter = 
		_a->m_body->GetPosition() + 
		linalg::mul(linalg::transpose(rotationMatrix)
		, (_b->m_body->GetPosition() - _a->m_body->GetPosition()));

	auto normal = rotatedCircleCenter - _a->m_body->GetPosition();
	auto closest = normal;

	float x_half_extent = _a->m_extent.x / 2.0f;
	float y_half_extent = _a->m_extent.y / 2.0f;

	closest.x = std::clamp(closest.x, -x_half_extent, x_half_extent);
	closest.y = std::clamp(closest.y, -y_half_extent, y_half_extent);

	bool inside = false;

	if (normal == closest)
	{
		inside = true;

		if (std::abs(normal.x) > std::abs(normal.y))
		{
			if (closest.x > 0)
				closest.x = x_half_extent;
			else
				closest.x = -x_half_extent;
		}
		else
		{
			if (closest.y > 0)
				closest.y = y_half_extent;
			else
				closest.y = -y_half_extent;
		}
	}

	normal = normal - closest;
	float d = linalg::length2(normal);
	float r = _b->m_radius;

	bool isHit = true;
	if (d > r * r && inside == false)
	{
		isHit = false;
	}

	// if inside, d < r
	d = std::sqrt(d);

	normal = (inside == true) ? -normal : normal;
	float penetration = (inside == true) ? r + d : r - d;
	// transform the data back to world space
	normal = linalg::mul(rotationMatrix, normal);
	normal = linalg::normalize(normal);

	float2 contactPoint = (inside == true) ? _b->m_body->GetPosition() : _b->m_body->GetPosition() - _b->m_radius * normal;

	return Manifold(
		_a->m_body,
		_b->m_body,
		contactPoint,
		normal,
		penetration,
		isHit
	);
}