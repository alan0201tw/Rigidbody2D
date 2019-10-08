#include "integrator.hpp"

#include <iostream>

void ExplicitEulerIntegrator::Integrate(const std::vector<BodyRef>& _bodies, float deltaTime)
{
    // from 2009 Erin Catto http://www.gphysics.com
    /* 
    state2->t = state1->t + h;
	state2->x0 = state1->x;
	state2->x = state1->x + h * state1->v;
	state2->v = state1->v - h * gravity;
    */
    for(size_t i = 0; i < _bodies.size(); i++)
    {
        // add gravity
        _bodies[i]->m_force += float2(0, -9.8f);

        _bodies[i]->m_position += deltaTime * _bodies[i]->m_velocity;
        _bodies[i]->m_velocity += deltaTime * _bodies[i]->m_force;

        _bodies[i]->m_force = float2(0, 0);

        // std::cout << "dt = " << deltaTime << std::endl;

        // std::cout << "_bodies[i]->m_position : " << _bodies[i]->m_position[0] << " " << _bodies[i]->m_position[1]
        //     << " , _bodies[i]->m_velocity : " << _bodies[i]->m_velocity[0]<< " " << _bodies[i]->m_velocity[1] << std::endl;
    }

    // TODO : we might need to add gravity somewhere.
    // maybe define the gravity vector in scene class.
}