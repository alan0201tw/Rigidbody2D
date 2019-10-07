#include "aabb.hpp"

#include "manifold.hpp"

Manifold AABB::accept(Shape::Visitor<Manifold>* visitor)
{
    return visitor->visitAABB(shared_from_this());
}

Manifold AABB::visitAABB(std::shared_ptr<AABB> _shape)
{
    // Exit with no intersection if found separated along an axis
    if(m_max[0] < _shape->m_min[0] or m_min[0] > _shape->m_max[0]) return false;
    if(m_max[1] < _shape->m_min[1] or m_min[1] > _shape->m_max[1]) return false;
    
    // No separating axis found, therefor there is at least one overlapping axis
    return Manifold(true);
}

Manifold AABB::visitCircle(std::shared_ptr<Circle> _shape)
{
    // TODO
    return Manifold(false);
}