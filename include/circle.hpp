#pragma once

#include "linalg.h"

#include "shape.hpp"

class Circle : public Shape
{
    typedef linalg::aliases::float2 float2;
private:
    float m_radius;

public:
    Circle(float _radius) : m_radius(_radius) {}

    virtual Manifold accept(const ShapeVisitor<Manifold>& visitor) const override;

    virtual Manifold visitAABB(const OBB& _shape) const override;
    virtual Manifold visitCircle(const Circle& _shape) const override;

    virtual void Render() const override;

    friend class CollisionHelper;
};