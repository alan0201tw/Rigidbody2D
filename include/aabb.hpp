#pragma once

#include "linalg.h"

#include "shape.hpp"

#include <vector>

class AABB : public Shape, public std::enable_shared_from_this<AABB>
{
    typedef linalg::aliases::float2 float2;
    typedef linalg::aliases::float2x2 float2x2;
private:
    float2 m_extent;

    float2 getSupportPoint(const float2& dir) const;
	static std::vector<Manifold> isVertexOfBInsideA(
		std::shared_ptr<const AABB> _a, std::shared_ptr<const AABB> _b);

public:
    AABB(float2 _extent) : m_extent(_extent) {}

    virtual Manifold accept(std::shared_ptr<const ShapeVisitor<Manifold>> visitor) const override;

    virtual Manifold visitAABB(std::shared_ptr<const AABB> _shape) const override;
    virtual Manifold visitCircle(std::shared_ptr<const Circle> _shape) const override;

    virtual void Render() const override;

    friend class CollisionHelper;
};