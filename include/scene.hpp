#pragma once

#include <vector>

#include "rigidbody2D.hpp"
#include "joint.hpp"
#include "integrator.hpp"

class Manifold;

class Scene
{
    typedef linalg::aliases::float2 float2;
private:

    typedef std::shared_ptr<RigidBody2D> BodyRef;
    typedef std::shared_ptr<Joint> JointRef;

    float m_deltaTime;
    uint32_t m_iterations;
    std::vector<BodyRef> m_bodies;
    std::vector<JointRef> m_joints;
    // this field should be updated by Step()
    mutable std::vector<Manifold> m_manifolds;

    std::shared_ptr<Integrator> m_integrator;

public:
    Scene(float _dt, uint32_t _iterations, std::shared_ptr<Integrator> _integrator) 
        : m_deltaTime(_dt), m_iterations(_iterations), m_bodies(), m_joints(),
          m_manifolds(), m_integrator(_integrator) {}

    void Step() const;
    void Render() const;
    // for a given shape, create a rigidbody and return it for further operation
    std::shared_ptr<RigidBody2D> AddRigidBody(std::shared_ptr<Shape> _shape, float2 _position);
    void AddJoint(std::shared_ptr<Joint> _joint);
};