#pragma once

#include "linmath.h"

#include "shape.hpp"

class AABB : public Shape, Shape::Visitor<Manifold>, std::enable_shared_from_this<AABB>
{
private:
    vec2 m_min, m_max;

public:
    virtual Manifold accept(Shape::Visitor<Manifold>* visitor) override;

    virtual Manifold visitAABB(std::shared_ptr<AABB> _shape) override;
    virtual Manifold visitCircle(std::shared_ptr<Circle> _shape) override;
};