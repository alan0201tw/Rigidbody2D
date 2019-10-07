#pragma once

#include "linalg.h"

#include "shape.hpp"

class AABB : public Shape, std::enable_shared_from_this<AABB>
{
    typedef linalg::aliases::float2 float2;
private:
    float2 m_min, m_max;

public:
    virtual Manifold accept(std::shared_ptr<ShapeVisitor<Manifold>> visitor) override;

    virtual Manifold visitAABB(std::shared_ptr<AABB> _shape) override;
    virtual Manifold visitCircle(std::shared_ptr<Circle> _shape) override;
};