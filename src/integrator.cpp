#include "integrator.hpp"

#include <algorithm>

void ExplicitEulerIntegrator::Integrate(const std::vector<BodyRef>& _bodies, float deltaTime)
{
    // from 2009 Erin Catto http://www.gphysics.com
    /* 
	state2->x = state1->x + h * state1->v;
	state2->v = state1->v - h * gravity;
    */
    for(size_t i = 0; i < _bodies.size(); i++)
    {
        if(_bodies[i]->m_mass == 0.0f)
            continue;

        // Linear
        _bodies[i]->m_position += deltaTime * _bodies[i]->m_velocity;
        // delta_v = delta_time * a = delta_time * F / m;
        _bodies[i]->m_velocity += deltaTime * (_bodies[i]->m_force / _bodies[i]->m_mass);
        // add gravity
        _bodies[i]->m_velocity += deltaTime * float2(0, -9.8f * 5);

        // Rotation
        _bodies[i]->m_orientation += _bodies[i]->m_angularVelocity * deltaTime;
        _bodies[i]->m_angularVelocity += deltaTime * (_bodies[i]->m_torque / _bodies[i]->m_inertia);

        _bodies[i]->m_force = float2(0, 0);
    }

    // TODO : we might need to add gravity somewhere.
    // maybe define the gravity vector in scene class.
}