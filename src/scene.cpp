#include "scene.hpp"

#include "manifold.hpp"
#include "shape.hpp"

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
    for(size_t i = 0; i < m_bodies.size(); i++)
    {
        // m_bodies[i]
    }
}

void Scene::Render() const
{

}

void Scene::AddRigidBody(std::shared_ptr<RigidBody2D> _body, float2 _position)
{

}