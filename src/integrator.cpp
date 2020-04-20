#include "integrator.hpp"

#include <algorithm>

#include "scene.hpp"

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
        _bodies[i]->AddAngularVelocity(deltaTime * (_bodies[i]->GetTorque() * _bodies[i]->GetInvInertia()));

        _bodies[i]->SetForce(float2(0, 0));
        _bodies[i]->SetTorque(0.0f);
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
        _bodies[i]->AddVelocity(deltaTime * (_bodies[i]->GetForce() * _bodies[i]->GetInvMass()));
        // add gravity
        _bodies[i]->AddVelocity(deltaTime * float2(0, -9.8f));

        // Rotation
        _bodies[i]->AddOrientation(_bodies[i]->GetAngularVelocity() * deltaTime);
        _bodies[i]->AddAngularVelocity(deltaTime * (_bodies[i]->GetTorque() * _bodies[i]->GetInvInertia()));

        _bodies[i]->SetForce(float2(0, 0));
        _bodies[i]->SetTorque(0.0f);
    }
}

void RungeKuttaFourthIntegrator::Integrate(const std::vector<BodyRef>& _bodies, float deltaTime)
{
	// this stores the absolute value of position and velocity
	std::vector<StateStep> currentState(_bodies.size());
	// below four arrays store the delta value of each state
	std::vector<StateStep> deltaK1State(_bodies.size());
	std::vector<StateStep> deltaK2State(_bodies.size());
	std::vector<StateStep> deltaK3State(_bodies.size());
	std::vector<StateStep> deltaK4State(_bodies.size());

	const float2 gravity(0.0f, -9.8f);

	for (size_t i = 0; i < _bodies.size(); i++)
	{
		if (_bodies[i]->GetInvMass() == 0.0f)
			continue;

		currentState[i].position = _bodies[i]->GetPosition();
		currentState[i].velocity = _bodies[i]->GetVelocity();
		currentState[i].orientation = _bodies[i]->GetOrientation();
		currentState[i].angularVelocity = _bodies[i]->GetAngularVelocity();
	}

	for (size_t i = 0; i < _bodies.size(); i++)
	{
		if (_bodies[i]->GetInvMass() == 0.0f)
			continue;

		const float2 acceleration = gravity + _bodies[i]->GetForce() * _bodies[i]->GetInvMass();
		const float angularAcc = _bodies[i]->GetTorque() * _bodies[i]->GetInvInertia();

		deltaK1State[i].velocity = acceleration * deltaTime;
		deltaK1State[i].position = _bodies[i]->GetVelocity() * deltaTime;
		deltaK1State[i].angularVelocity = angularAcc * deltaTime;
		deltaK1State[i].orientation = _bodies[i]->GetAngularVelocity() * deltaTime;

		//_bodies[i]->AddVelocity(deltaK1State[i].velocity * 0.5f);
		//_bodies[i]->AddPosition(deltaK1State[i].position * 0.5f);
		_bodies[i]->SetVelocity(currentState[i].velocity + deltaK1State[i].velocity * 0.5f);
		_bodies[i]->SetPosition(currentState[i].position + deltaK1State[i].position * 0.5f);

		_bodies[i]->AddAngularVelocity(deltaK1State[i].angularVelocity * 0.5f);
		_bodies[i]->AddOrientation(deltaK1State[i].orientation * 0.5f);
	}

	for (size_t i = 0; i < _bodies.size(); i++)
	{
		if (_bodies[i]->GetInvMass() == 0.0f)
			continue;

		const float2 acceleration = gravity + _bodies[i]->GetForce() * _bodies[i]->GetInvMass();
		const float angularAcc = _bodies[i]->GetTorque() * _bodies[i]->GetInvInertia();

		deltaK2State[i].velocity = acceleration * deltaTime;
		deltaK2State[i].position = _bodies[i]->GetVelocity() * deltaTime;
		deltaK2State[i].angularVelocity = angularAcc * deltaTime;
		deltaK2State[i].orientation = _bodies[i]->GetAngularVelocity() * deltaTime;

		//_bodies[i]->AddVelocity(deltaK2State[i].velocity * 0.5f);
		//_bodies[i]->AddPosition(deltaK2State[i].position * 0.5f);
		_bodies[i]->SetVelocity(currentState[i].velocity + deltaK2State[i].velocity * 0.5f);
		_bodies[i]->SetPosition(currentState[i].position + deltaK2State[i].position * 0.5f);

		_bodies[i]->AddAngularVelocity(deltaK2State[i].angularVelocity * 0.5f);
		_bodies[i]->AddOrientation(deltaK2State[i].orientation * 0.5f);
	}

	for (size_t i = 0; i < _bodies.size(); i++)
	{
		if (_bodies[i]->GetInvMass() == 0.0f)
			continue;

		const float2 acceleration = gravity + _bodies[i]->GetForce() * _bodies[i]->GetInvMass();
		const float angularAcc = _bodies[i]->GetTorque() * _bodies[i]->GetInvInertia();

		deltaK3State[i].velocity = acceleration * deltaTime;
		deltaK3State[i].position = _bodies[i]->GetVelocity() * deltaTime;
		deltaK3State[i].angularVelocity = angularAcc * deltaTime;
		deltaK3State[i].orientation = _bodies[i]->GetAngularVelocity() * deltaTime;

		//_bodies[i]->AddVelocity(deltaK3State[i].velocity);
		//_bodies[i]->AddPosition(deltaK3State[i].position);
		_bodies[i]->SetVelocity(currentState[i].velocity + deltaK3State[i].velocity * 0.5f);
		_bodies[i]->SetPosition(currentState[i].position + deltaK3State[i].position * 0.5f);
		_bodies[i]->AddAngularVelocity(deltaK3State[i].angularVelocity * 0.5f);
		_bodies[i]->AddOrientation(deltaK3State[i].orientation * 0.5f);
	}

	for (size_t i = 0; i < _bodies.size(); i++)
	{
		if (_bodies[i]->GetInvMass() == 0.0f)
			continue;

		const float2 acceleration = gravity + _bodies[i]->GetForce() * _bodies[i]->GetInvMass();
		const float angularAcc = _bodies[i]->GetTorque() * _bodies[i]->GetInvInertia();

		deltaK4State[i].velocity = acceleration * deltaTime;
		deltaK4State[i].position = _bodies[i]->GetVelocity() * deltaTime;
		deltaK4State[i].angularVelocity = angularAcc * deltaTime;
		deltaK4State[i].orientation = _bodies[i]->GetAngularVelocity() * deltaTime;
	}

	// final integration
	for (size_t i = 0; i < _bodies.size(); i++)
	{
		if (_bodies[i]->GetInvMass() == 0.0f)
			continue;

		float2 deltaPos =
			(deltaK1State[i].position + 2.0f * deltaK2State[i].position +
				2.0f * deltaK3State[i].position + deltaK4State[i].position) / 6.0f;
		_bodies[i]->SetPosition(currentState[i].position + deltaPos);

		float2 deltaVel =
			(deltaK1State[i].velocity + 2.0f * deltaK2State[i].velocity +
				2.0f * deltaK3State[i].velocity + deltaK4State[i].velocity) / 6.0f;
		_bodies[i]->SetVelocity(currentState[i].velocity + deltaVel);

		float deltaOri =
			(deltaK1State[i].orientation + 2.0f * deltaK2State[i].orientation +
				2.0f * deltaK3State[i].orientation + deltaK4State[i].orientation) / 6.0f;
		_bodies[i]->SetOrientation(currentState[i].orientation + deltaOri);

		float deltaAngVel =
			(deltaK1State[i].angularVelocity + 2.0f * deltaK2State[i].angularVelocity +
				2.0f * deltaK3State[i].angularVelocity + deltaK4State[i].angularVelocity) / 6.0f;
		_bodies[i]->SetAngularVelocity(currentState[i].angularVelocity + deltaAngVel);

		_bodies[i]->SetForce(float2(0, 0));
		_bodies[i]->SetTorque(0.0f);
	}
}
