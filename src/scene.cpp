#include "scene.hpp"

#include "manifold.hpp"
#include "shape.hpp"
#include "integrator.hpp"

#include "GL/freeglut.h"

#include <iostream>

void Scene::Step() const
{
	Solve();
	Integrate();
}

void Scene::Solve() const
{
	// First : Generate manifolds
	for (size_t i = 0; i < m_bodies.size(); i++)
	{
		for (size_t j = i + 1; j < m_bodies.size(); j++)
		{
			Manifold manifold =
				m_bodies[i]->GetShape()->accept(m_bodies[j]->GetShape());

			// put the manifold into resolve queue only if it's a hit
			if (manifold.m_isHit == true)
				m_manifolds.push_back(manifold);
		}
	}

	// Then : Resolve impulses by manifolds
	for (size_t iteration = 0; iteration < m_iterations; iteration++)
	{
		for (size_t i = 0; i < m_manifolds.size(); i++)
		{
			m_manifolds[i].Resolve();
		}
	}

	// Then : Do positional correction
	for (size_t i = 0; i < m_manifolds.size(); i++)
	{
		m_manifolds[i].PositionalCorrection();
	}

	// Preprocess : apply joint constraint
	for (size_t i = 0; i < m_joints.size(); i++)
	{
		m_joints[i]->ApplyConstriant();
	}

	// Remember to clear the manifolds
	m_manifolds.clear();
}

void Scene::Integrate() const
{
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
        if(m_manifolds[i].m_isHit == false)
            continue;
        for(int k = 0; k < m_manifolds[i].m_contactPointCount; ++k)
        {
            // render contact point
            glPushAttrib(GL_CURRENT_BIT);
            glPointSize( 4.0f );
            glBegin(GL_POINTS);
            {
                glPushMatrix();
                
                glColor3f(1.0f, 0.0f, 0.0f);

                glVertex2f(m_manifolds[i].m_contactPoints[k].x, 
                    m_manifolds[i].m_contactPoints[k].y);

                glPopMatrix();
            }
            glEnd();
            glPointSize( 1.0f );
            glPopAttrib();
            // render normal
            glPushAttrib(GL_CURRENT_BIT);
            glBegin(GL_LINE_STRIP);
            {
                glPushMatrix();
                
                glColor3f(0.0f, 1.0f, 0.3f);

                glVertex2f(m_manifolds[i].m_contactPoints[k].x, 
                    m_manifolds[i].m_contactPoints[k].y);

                glVertex2f(m_manifolds[i].m_contactPoints[k].x + m_manifolds[i].m_normal.x, 
                    m_manifolds[i].m_contactPoints[k].y + m_manifolds[i].m_normal.y);

                glPopMatrix();
            }
            glEnd();
            glPopAttrib();
        }
    }
    
    m_manifolds.clear();
}

std::shared_ptr<RigidBody2D> Scene::AddRigidBody(std::shared_ptr<Shape> _shape, float2 _position)
{
    if(_shape->m_body != nullptr)
    {
        throw std::runtime_error("Error : Scene::AddRigidBody : Trying to reuse shape!");
        // std::cerr << "Error : Scene::AddRigidBody : Trying to reuse shape!" << std::endl;
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