#pragma once

#include <memory>

class Manifold;

class Shape
{
public:
    template <typename R> class Visitor;

    // Here the return type of 'bool' is just a placeholder.
    // We will replace it with a data structure for storing collision data.
    // A so-called 'Manifold'.
    virtual Manifold accept(Visitor<Manifold>* visitor) = 0;
};

// here we need to forward declare all sub-classes of 'Shape'
class AABB;
class Circle;

template <typename R>
class Shape::Visitor
{
public:
    virtual R visitAABB(std::shared_ptr<AABB> _shape) = 0;
    virtual R visitCircle(std::shared_ptr<Circle> _shape) = 0;
};