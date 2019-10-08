#pragma once

#include <memory>

#include "manifold.hpp"
class RigidBody2D;

// here we need to forward declare all sub-classes of 'Shape'
class AABB;
class Circle;

template <typename R>
class ShapeVisitor
{
public:
    virtual R visitAABB(std::shared_ptr<AABB> _shape) = 0;
    virtual R visitCircle(std::shared_ptr<Circle> _shape) = 0;
};

class Shape : public ShapeVisitor<Manifold>
{
public:
    std::shared_ptr<RigidBody2D> m_body;
public:
    // Here the return type of 'bool' is just a placeholder.
    // We will replace it with a data structure for storing collision data.
    // A so-called 'Manifold'.
    virtual Manifold accept(std::shared_ptr<ShapeVisitor<Manifold>> visitor) = 0;

    // Following sections are for rendering, it is more sophisticated to
    // decouple these two behaviors, but for the sake of convenience, we
    // will just do it here.
    virtual void Render() = 0;
};