#include "circle.hpp"

#include "manifold.hpp"

Manifold Circle::accept(std::shared_ptr<ShapeVisitor<Manifold>> visitor)
{
    return visitor->visitCircle(shared_from_this());
}

Manifold Circle::visitAABB(std::shared_ptr<AABB> _shape)
{
    // TODO
    return Manifold(false);
}

Manifold Circle::visitCircle(std::shared_ptr<Circle> _shape)
{
    float2 position = m_body->GetPosition();
    float2 otherPosition = _shape->m_body->GetPosition();

    float radius_sum_sqr = m_radius + _shape->m_radius;
    radius_sum_sqr *= radius_sum_sqr;

    float x_sum = (position[0] + otherPosition[0]);
    float x_sum_sqr = x_sum * x_sum;

    float y_sum = (position[1] + otherPosition[1]);
    float y_sum_sqr = y_sum * y_sum;

    bool isColliding = (radius_sum_sqr < x_sum_sqr + y_sum_sqr);
    return Manifold(isColliding);
}