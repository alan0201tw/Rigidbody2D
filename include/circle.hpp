#pragma once

#include "linalg.h"

#include "shape.hpp"

class Circle : public Shape, std::enable_shared_from_this<Circle>
{
    typedef linalg::aliases::float2 float2;
private:
    float m_radius;

public:
    virtual Manifold accept(std::shared_ptr<ShapeVisitor<Manifold>> visitor) override;

    virtual Manifold visitAABB(std::shared_ptr<AABB> _shape) override;
    virtual Manifold visitCircle(std::shared_ptr<Circle> _shape) override;
};