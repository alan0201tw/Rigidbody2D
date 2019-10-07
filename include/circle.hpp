#pragma once

#include "linmath.h"

#include "shape.hpp"

class Circle : public Shape, Shape::Visitor<Manifold>, std::enable_shared_from_this<Circle>
{
private:
    float m_radius;
    vec3 m_position;

public:
    virtual Manifold accept(Shape::Visitor<Manifold>* visitor) override;

    virtual Manifold visitAABB(std::shared_ptr<AABB> _shape) override;
    virtual Manifold visitCircle(std::shared_ptr<Circle> _shape) override;
};