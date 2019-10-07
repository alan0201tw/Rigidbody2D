#pragma once

#include <vector>

#include "rigidbody2D.hpp"

class Manifold;

class Scene
{
    typedef linalg::aliases::float2 float2;
private:

    typedef std::shared_ptr<RigidBody2D> BodyRef;

    float m_deltaTime;
    uint m_iterations;
    std::vector<BodyRef> m_bodies;
    // this field should be updated by Step()
    mutable std::vector<Manifold> m_manifolds;

public:
    Scene(float _dt) : m_deltaTime(_dt) {}

    void Step() const;
    void Render() const;
    // for a given shape, create a rigidbody and return it for further operation
    std::shared_ptr<RigidBody2D> AddRigidBody(std::shared_ptr<Shape> _shape, float2 _position);
};