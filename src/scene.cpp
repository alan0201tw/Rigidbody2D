#include "scene.hpp"

#include "manifold.hpp"
#include "shape.hpp"
#include "integrator.hpp"

void Scene::Step() const
{
    for(size_t i = 0; i < m_bodies.size(); i++)
    {
        for(size_t j = i + 1; j < m_bodies.size(); j++)
        {
            Manifold manifold = 
                m_bodies[i]->GetShape()->accept(m_bodies[j]->GetShape());

            m_manifolds.push_back(manifold);
        }
    }

    for(size_t i = 0; i < m_manifolds.size(); i++)
    {
        m_manifolds[i].Resolve();
    }

    // integrate
    static ExplicitEulerIntegrator integrator;
    integrator.Integrate(m_bodies, m_deltaTime);
}

void Scene::Render() const
{
    for(size_t i = 0; i < m_bodies.size(); i++)
    {
        m_bodies[i]->GetShape()->Render();
    }
}

std::shared_ptr<RigidBody2D> Scene::AddRigidBody(std::shared_ptr<Shape> _shape, float2 _position)
{
    std::shared_ptr<RigidBody2D> body = 
        std::make_shared<RigidBody2D>(_shape, _position, 0.2f, 1.0f);

    _shape->m_body = body;

    m_bodies.push_back(body);
    return body;
}