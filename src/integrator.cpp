#include "integrator.hpp"

#include <algorithm>

#include "scene.hpp"

void ExplicitEulerIntegrator::Integrate(std::shared_ptr<Scene> scene)
{
    // from 2009 Erin Catto http://www.gphysics.com
    /* 
	state2->x = state1->x + h * state1->v;
	state2->v = state1->v - h * gravity;
    */
    for(size_t i = 0; i < scene->m_bodies.size(); i++)
    {
        if(scene->m_bodies[i]->GetInvMass() == 0.0f)
            continue;

        // Linear
        scene->m_bodies[i]->AddPosition(scene->m_deltaTime * scene->m_bodies[i]->GetVelocity());
        // delta_v = delta_time * a = delta_time * F / m;
        scene->m_bodies[i]->AddVelocity(scene->m_deltaTime * (scene->m_bodies[i]->GetForce() / scene->m_bodies[i]->GetMass()));
        // add gravity
        scene->m_bodies[i]->AddVelocity(scene->m_deltaTime * float2(0, -9.8f));

        // Rotation
        scene->m_bodies[i]->AddOrientation(scene->m_bodies[i]->GetAngularVelocity() * scene->m_deltaTime);
        scene->m_bodies[i]->AddAngularVelocity(scene->m_deltaTime * (scene->m_bodies[i]->GetTorque() * scene->m_bodies[i]->GetInvInertia()));

        scene->m_bodies[i]->SetForce(float2(0, 0));
        scene->m_bodies[i]->SetTorque(0.0f);
    }

    // TODO : we might need to add gravity somewhere.
    // maybe define the gravity vector in scene class.
}


void SymplecticEulerIntegrator::Integrate(std::shared_ptr<Scene> scene)
{
	// from 2009 Erin Catto http://www.gphysics.com
	/*
	state2->v = state1->v - h * gravity;
	state2->x = state1->x + h * state2->v;
	*/
	for (size_t i = 0; i < scene->m_bodies.size(); i++)
	{
		if (scene->m_bodies[i]->GetInvMass() == 0.0f)
			continue;

		scene->m_bodies[i]->AddVelocity(scene->m_deltaTime * (scene->m_bodies[i]->GetForce() / scene->m_bodies[i]->GetMass()));
		scene->m_bodies[i]->AddVelocity(scene->m_deltaTime * float2(0, -9.8f));

		scene->m_bodies[i]->AddPosition(scene->m_deltaTime * scene->m_bodies[i]->GetVelocity());

		// Rotation
		scene->m_bodies[i]->AddAngularVelocity(scene->m_deltaTime * (scene->m_bodies[i]->GetTorque() * scene->m_bodies[i]->GetInvInertia()));
		scene->m_bodies[i]->AddOrientation(scene->m_bodies[i]->GetAngularVelocity() * scene->m_deltaTime);

		scene->m_bodies[i]->SetForce(float2(0, 0));
		scene->m_bodies[i]->SetTorque(0.0f);
	}
}


void NewtonIntegrator::Integrate(std::shared_ptr<Scene> scene)
{
    // from 2009 Erin Catto http://www.gphysics.com
    /* 
	state2->x = state1->x + h * state1->v - 0.5f * h * h * gravity;
	state2->v = state1->v - h * gravity;
    */

    const float2 gravity(0.0f, -9.8f);

    for(size_t i = 0; i < scene->m_bodies.size(); i++)
    {
        if(scene->m_bodies[i]->GetInvMass() == 0.0f)
            continue;

        // Linear
        scene->m_bodies[i]->AddPosition(scene->m_deltaTime * scene->m_bodies[i]->GetVelocity() + 0.5f * scene->m_deltaTime * scene->m_deltaTime * gravity);
        // delta_v = delta_time * a = delta_time * F / m;
        scene->m_bodies[i]->AddVelocity(scene->m_deltaTime * (scene->m_bodies[i]->GetForce() * scene->m_bodies[i]->GetInvMass()));
        // add gravity
        scene->m_bodies[i]->AddVelocity(scene->m_deltaTime * float2(0, -9.8f));

        // Rotation
        scene->m_bodies[i]->AddOrientation(scene->m_bodies[i]->GetAngularVelocity() * scene->m_deltaTime);
        scene->m_bodies[i]->AddAngularVelocity(scene->m_deltaTime * (scene->m_bodies[i]->GetTorque() * scene->m_bodies[i]->GetInvInertia()));

        scene->m_bodies[i]->SetForce(float2(0, 0));
        scene->m_bodies[i]->SetTorque(0.0f);
    }
}

void RungeKuttaFourthIntegrator::Integrate(std::shared_ptr<Scene> scene)
{
	// this stores the absolute value of position and velocity
	std::vector<StateStep> currentState(scene->m_bodies.size());
	// below four arrays store the delta value of each state
	std::vector<StateStep> deltaK1State(scene->m_bodies.size());
	std::vector<StateStep> deltaK2State(scene->m_bodies.size());
	std::vector<StateStep> deltaK3State(scene->m_bodies.size());
	std::vector<StateStep> deltaK4State(scene->m_bodies.size());

	const float2 gravity(0.0f, -9.8f);

	for (size_t i = 0; i < scene->m_bodies.size(); i++)
	{
		if (scene->m_bodies[i]->GetInvMass() == 0.0f)
			continue;

		currentState[i].position = scene->m_bodies[i]->GetPosition();
		currentState[i].velocity = scene->m_bodies[i]->GetVelocity();
		currentState[i].orientation = scene->m_bodies[i]->GetOrientation();
		currentState[i].angularVelocity = scene->m_bodies[i]->GetAngularVelocity();
	}

	scene->Solve();

	for (size_t i = 0; i < scene->m_bodies.size(); i++)
	{
		if (scene->m_bodies[i]->GetInvMass() == 0.0f)
			continue;

		const float2 acceleration = gravity + scene->m_bodies[i]->GetForce() * scene->m_bodies[i]->GetInvMass();
		const float angularAcc = scene->m_bodies[i]->GetTorque() * scene->m_bodies[i]->GetInvInertia();

		deltaK1State[i].velocity = acceleration * scene->m_deltaTime;
		deltaK1State[i].position = scene->m_bodies[i]->GetVelocity() * scene->m_deltaTime;
		deltaK1State[i].angularVelocity = angularAcc * scene->m_deltaTime;
		deltaK1State[i].orientation = scene->m_bodies[i]->GetAngularVelocity() * scene->m_deltaTime;

		scene->m_bodies[i]->SetVelocity(currentState[i].velocity + deltaK1State[i].velocity * 0.5f);
		scene->m_bodies[i]->SetPosition(currentState[i].position + deltaK1State[i].position * 0.5f);
		scene->m_bodies[i]->SetAngularVelocity(currentState[i].angularVelocity + deltaK1State[i].angularVelocity * 0.5f);
		scene->m_bodies[i]->SetOrientation(currentState[i].orientation + deltaK1State[i].orientation * 0.5f);

		scene->m_bodies[i]->SetForce(float2(0.0f, 0.0f));
		scene->m_bodies[i]->SetTorque(0.0f);
	}

	scene->Solve();

	for (size_t i = 0; i < scene->m_bodies.size(); i++)
	{
		if (scene->m_bodies[i]->GetInvMass() == 0.0f)
			continue;

		const float2 acceleration = gravity + scene->m_bodies[i]->GetForce() * scene->m_bodies[i]->GetInvMass();
		const float angularAcc = scene->m_bodies[i]->GetTorque() * scene->m_bodies[i]->GetInvInertia();

		deltaK2State[i].velocity = acceleration * scene->m_deltaTime;
		deltaK2State[i].position = scene->m_bodies[i]->GetVelocity() * scene->m_deltaTime;
		deltaK2State[i].angularVelocity = angularAcc * scene->m_deltaTime;
		deltaK2State[i].orientation = scene->m_bodies[i]->GetAngularVelocity() * scene->m_deltaTime;

		scene->m_bodies[i]->SetVelocity(currentState[i].velocity + deltaK2State[i].velocity * 0.5f);
		scene->m_bodies[i]->SetPosition(currentState[i].position + deltaK2State[i].position * 0.5f);
		scene->m_bodies[i]->SetAngularVelocity(currentState[i].angularVelocity + deltaK2State[i].angularVelocity * 0.5f);
		scene->m_bodies[i]->SetOrientation(currentState[i].orientation + deltaK2State[i].orientation * 0.5f);

		scene->m_bodies[i]->SetForce(float2(0.0f, 0.0f));
		scene->m_bodies[i]->SetTorque(0.0f);
	}

	scene->Solve();

	for (size_t i = 0; i < scene->m_bodies.size(); i++)
	{
		if (scene->m_bodies[i]->GetInvMass() == 0.0f)
			continue;

		const float2 acceleration = gravity + scene->m_bodies[i]->GetForce() * scene->m_bodies[i]->GetInvMass();
		const float angularAcc = scene->m_bodies[i]->GetTorque() * scene->m_bodies[i]->GetInvInertia();

		deltaK3State[i].velocity = acceleration * scene->m_deltaTime;
		deltaK3State[i].position = scene->m_bodies[i]->GetVelocity() * scene->m_deltaTime;
		deltaK3State[i].angularVelocity = angularAcc * scene->m_deltaTime;
		deltaK3State[i].orientation = scene->m_bodies[i]->GetAngularVelocity() * scene->m_deltaTime;

		scene->m_bodies[i]->SetVelocity(currentState[i].velocity + deltaK3State[i].velocity);
		scene->m_bodies[i]->SetPosition(currentState[i].position + deltaK3State[i].position);
		scene->m_bodies[i]->SetAngularVelocity(currentState[i].angularVelocity + deltaK3State[i].angularVelocity);
		scene->m_bodies[i]->SetOrientation(currentState[i].orientation + deltaK3State[i].orientation);

		scene->m_bodies[i]->SetForce(float2(0.0f, 0.0f));
		scene->m_bodies[i]->SetTorque(0.0f);
	}

	scene->m_deltaTime = scene->m_deltaTime;
	scene->Solve();

	for (size_t i = 0; i < scene->m_bodies.size(); i++)
	{
		if (scene->m_bodies[i]->GetInvMass() == 0.0f)
			continue;

		const float2 acceleration = gravity + scene->m_bodies[i]->GetForce() * scene->m_bodies[i]->GetInvMass();
		const float angularAcc = scene->m_bodies[i]->GetTorque() * scene->m_bodies[i]->GetInvInertia();

		deltaK4State[i].velocity = acceleration * scene->m_deltaTime;
		deltaK4State[i].position = scene->m_bodies[i]->GetVelocity() * scene->m_deltaTime;
		deltaK4State[i].angularVelocity = angularAcc * scene->m_deltaTime;
		deltaK4State[i].orientation = scene->m_bodies[i]->GetAngularVelocity() * scene->m_deltaTime;
	}

	// final integration
	for (size_t i = 0; i < scene->m_bodies.size(); i++)
	{
		if (scene->m_bodies[i]->GetInvMass() == 0.0f)
			continue;

		float2 deltaPos =
			(deltaK1State[i].position + 2.0f * deltaK2State[i].position +
				2.0f * deltaK3State[i].position + deltaK4State[i].position) / 6.0f;
		scene->m_bodies[i]->SetPosition(currentState[i].position + deltaPos);

		float2 deltaVel =
			(deltaK1State[i].velocity + 2.0f * deltaK2State[i].velocity +
				2.0f * deltaK3State[i].velocity + deltaK4State[i].velocity) / 6.0f;
		scene->m_bodies[i]->SetVelocity(currentState[i].velocity + deltaVel);

		float deltaOri =
			(deltaK1State[i].orientation + 2.0f * deltaK2State[i].orientation +
				2.0f * deltaK3State[i].orientation + deltaK4State[i].orientation) / 6.0f;
		scene->m_bodies[i]->SetOrientation(currentState[i].orientation + deltaOri);

		float deltaAngVel =
			(deltaK1State[i].angularVelocity + 2.0f * deltaK2State[i].angularVelocity +
				2.0f * deltaK3State[i].angularVelocity + deltaK4State[i].angularVelocity) / 6.0f;
		scene->m_bodies[i]->SetAngularVelocity(currentState[i].angularVelocity + deltaAngVel);

		scene->m_bodies[i]->SetForce(float2(0, 0));
		scene->m_bodies[i]->SetTorque(0.0f);
	}
}
