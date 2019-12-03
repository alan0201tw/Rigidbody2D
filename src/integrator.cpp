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
        if(_bodies[i]->GetInvMass() == 0.0f)
            continue;

        // Linear
        _bodies[i]->AddPosition(deltaTime * _bodies[i]->GetVelocity());
        // delta_v = delta_time * a = delta_time * F / m;
        _bodies[i]->AddVelocity(deltaTime * (_bodies[i]->GetForce() / _bodies[i]->GetMass()));
        // add gravity
        _bodies[i]->AddVelocity(deltaTime * float2(0, -9.8f));

        // Rotation
        _bodies[i]->AddOrientation(_bodies[i]->GetAngularVelocity() * deltaTime);
        _bodies[i]->AddAngularVelocity(deltaTime * (_bodies[i]->GetTorque() / _bodies[i]->GetInertia()));

        _bodies[i]->SetForce(float2(0, 0));
    }

    // TODO : we might need to add gravity somewhere.
    // maybe define the gravity vector in scene class.
}

void NewtonIntegrator::Integrate(const std::vector<BodyRef>& _bodies, float deltaTime)
{
    // from 2009 Erin Catto http://www.gphysics.com
    /* 
	state2->x = state1->x + h * state1->v - 0.5f * h * h * gravity;
	state2->v = state1->v - h * gravity;
    */

    const float2 gravity(0.0f, -9.8f);

    for(size_t i = 0; i < _bodies.size(); i++)
    {
        if(_bodies[i]->GetInvMass() == 0.0f)
            continue;

        // Linear
        _bodies[i]->AddPosition(deltaTime * _bodies[i]->GetVelocity() + 0.5f * deltaTime * deltaTime * gravity);
        // delta_v = delta_time * a = delta_time * F / m;
        _bodies[i]->AddVelocity(deltaTime * (_bodies[i]->GetForce() / _bodies[i]->GetMass()));
        // add gravity
        _bodies[i]->AddVelocity(deltaTime * float2(0, -9.8f));

        // Rotation
        _bodies[i]->AddOrientation(_bodies[i]->GetAngularVelocity() * deltaTime);
        _bodies[i]->AddAngularVelocity(deltaTime * (_bodies[i]->GetTorque() / _bodies[i]->GetInertia()));

        _bodies[i]->SetForce(float2(0, 0));
    }

    // TODO : we might need to add gravity somewhere.
    // maybe define the gravity vector in scene class.
}