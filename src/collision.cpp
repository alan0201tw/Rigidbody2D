#include "collision.hpp"

#include "util.hpp"

#include <algorithm>
#include <iostream>

#include "linalg.h"

Manifold CollisionHelper::GenerateManifold(std::shared_ptr<const OBB> _a, std::shared_ptr<const Circle> _b)
{
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

	if (normal != closest)
		normal = normal - closest;

	float d = linalg::length2(normal);
	float r = _b->m_radius;

	bool isHit = true;
	if (d > r * r && inside == false)
	{
		isHit = false;
	}

	// if inside, d < r
	if (d != 0.0f)
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
		(isHit == true) ? 1 : 0,
		{ contactPoint },
		normal,
		penetration,
		isHit
	);
}