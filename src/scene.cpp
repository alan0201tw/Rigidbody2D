#include "scene.hpp"

#include "manifold.hpp"
#include "shape.hpp"
#include "integrator.hpp"

#include <iostream>

void Scene::Step() const
{
    // First : Generate manifolds
    for(size_t i = 0; i < m_bodies.size(); i++)
    {
        for(size_t j = i + 1; j < m_bodies.size(); j++)
        {
            Manifold manifold = 
                m_bodies[i]->GetShape()->accept(m_bodies[j]->GetShape());

            m_manifolds.push_back(manifold);
        }
    }
    // Then : Resolve impulses by manifolds
    for(size_t iteration = 0; iteration < m_iterations; iteration++)
    {
        for(size_t i = 0; i < m_manifolds.size(); i++)
        {
            m_manifolds[i].Resolve();
        }
    }
    // Then : Do positional correction
    for(size_t i = 0; i < m_manifolds.size(); i++)
    {
        m_manifolds[i].PositionalCorrection();
    }

    for(size_t i = 0; i < m_joints.size(); i++)
    {
        m_joints[i]->ApplyConstriant();
    }

    // Remember to clear the manifolds
    m_manifolds.clear();

    // integrate
    m_integrator->Integrate(m_bodies, m_deltaTime);
}

void Scene::Render() const
{
    for(size_t i = 0; i < m_bodies.size(); i++)
    {
        m_bodies[i]->GetShape()->Render();
    }
    for(size_t i = 0; i < m_joints.size(); i++)
    {
        m_joints[i]->Render();
    }
}

std::shared_ptr<RigidBody2D> Scene::AddRigidBody(std::shared_ptr<Shape> _shape, float2 _position)
{
    if(_shape->m_body != nullptr)
    {
        std::cerr << "Scene::AddRigidBody : Error trying to reuse shape!" << std::endl;
        return nullptr;
    }

    std::shared_ptr<RigidBody2D> body = 
        std::make_shared<RigidBody2D>(_shape, _position, 0.2f, 1.0f, 0.5f, 0.3f);

    _shape->m_body = body;

    m_bodies.push_back(body);
    return body;
}

void Scene::AddJoint(std::shared_ptr<Joint> _joint)
{
    m_joints.push_back(_joint);
}