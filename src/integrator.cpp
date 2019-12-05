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
        // _bodies[i]->AddOrientation(_bodies[i]->GetAngularVelocity() * deltaTime);
        // _bodies[i]->AddAngularVelocity(deltaTime * (_bodies[i]->GetTorque() / _bodies[i]->GetInertia()));

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
        _bodies[i]->AddVelocity(deltaTime * (_bodies[i]->GetForce() * _bodies[i]->GetInvMass()));
        // add gravity
        _bodies[i]->AddVelocity(deltaTime * float2(0, -9.8f));

        // Rotation
        // _bodies[i]->AddOrientation(_bodies[i]->GetAngularVelocity() * deltaTime);
        // _bodies[i]->AddAngularVelocity(deltaTime * (_bodies[i]->GetTorque() / _bodies[i]->GetInertia()));

        _bodies[i]->SetForce(float2(0, 0));
    }

    // TODO : we might need to add gravity somewhere.
    // maybe define the gravity vector in scene class.
}

void RungeKuttaFourthIntegrator::Integrate(const std::vector<BodyRef>& _bodies, float deltaTime)
{
    if(scene == nullptr)
    {
        throw std::runtime_error("RungeKuttaFourthIntegrator has no target scene.");
    }

    // this stores the absolute value of position and velocity
    StateStep currentState[_bodies.size()];
    // below four arrays store the delta value of each state
    StateStep deltaK1State[_bodies.size()];
    StateStep deltaK2State[_bodies.size()];
    StateStep deltaK3State[_bodies.size()];
    StateStep deltaK4State[_bodies.size()];

    // when using vectors
    // currentState.reserve(_bodies.size());
    // k1State.reserve(_bodies.size());
    // k2State.reserve(_bodies.size());
    // k3State.reserve(_bodies.size());
    // k4State.reserve(_bodies.size());

    // use this temprorary ExplicitEulerIntegrator to do the integration
    // required in RK4
    const auto originalIntegrator = scene->m_integrator;
    const auto originalDeltaTime = scene->m_deltaTime;
    scene->m_integrator = std::make_shared<ExplicitEulerIntegrator>();

    const float2 gravity(0.0f, -9.8f);

    for(size_t i = 0; i < _bodies.size(); i++)
    {
        if(_bodies[i]->GetInvMass() == 0.0f)
            continue;
        
        currentState[i].position = _bodies[i]->GetPosition();
        currentState[i].velocity = _bodies[i]->GetVelocity();
    }

    scene->m_deltaTime = originalDeltaTime;
    scene->Step();

    for(size_t i = 0; i < _bodies.size(); i++)
    {
        if(_bodies[i]->GetInvMass() == 0.0f)
            continue;
            
        const float2 acceleration = gravity + _bodies[i]->GetForce() * _bodies[i]->GetInvMass();

        deltaK1State[i].velocity = acceleration * scene->m_deltaTime;
        deltaK1State[i].position = _bodies[i]->GetVelocity() * scene->m_deltaTime;
    }

    scene->m_deltaTime = originalDeltaTime * 0.5f;
    scene->Step();

    for(size_t i = 0; i < _bodies.size(); i++)
    {
        if(_bodies[i]->GetInvMass() == 0.0f)
            continue;
        
        const float2 acceleration = gravity + _bodies[i]->GetForce() * _bodies[i]->GetInvMass();

        deltaK2State[i].velocity = acceleration * scene->m_deltaTime;
        deltaK2State[i].position = _bodies[i]->GetVelocity() * scene->m_deltaTime;
    }

    scene->m_deltaTime = originalDeltaTime * 0.5f;
    scene->Step();

    for(size_t i = 0; i < _bodies.size(); i++)
    {
        if(_bodies[i]->GetInvMass() == 0.0f)
            continue;
        
        const float2 acceleration = gravity + _bodies[i]->GetForce() * _bodies[i]->GetInvMass();

        deltaK3State[i].velocity = acceleration * scene->m_deltaTime;
        deltaK3State[i].position = _bodies[i]->GetVelocity() * scene->m_deltaTime;
    }

    scene->m_deltaTime = originalDeltaTime;
    scene->Step();

    for(size_t i = 0; i < _bodies.size(); i++)
    {
        if(_bodies[i]->GetInvMass() == 0.0f)
            continue;
        
        const float2 acceleration = gravity + _bodies[i]->GetForce() * _bodies[i]->GetInvMass();

        deltaK4State[i].velocity = acceleration * scene->m_deltaTime;
        deltaK4State[i].position = _bodies[i]->GetVelocity() * scene->m_deltaTime;
    }

    // final integration
    for(size_t i = 0; i < _bodies.size(); i++)
    {
        if(_bodies[i]->GetInvMass() == 0.0f)
            continue;

        float2 deltaPos = 
            (deltaK1State[i].position + 2.0f * deltaK2State[i].position + 
            2.0f * deltaK3State[i].position + deltaK4State[i].position) / 6.0f;
        _bodies[i]->SetPosition(currentState[i].position + deltaPos);
        
        float2 deltaVel = 
            (deltaK1State[i].velocity + 2.0f * deltaK2State[i].velocity + 
            2.0f * deltaK3State[i].velocity + deltaK4State[i].velocity) / 6.0f;
        _bodies[i]->SetVelocity(currentState[i].velocity + deltaVel);

        // add gravity
        // _bodies[i]->AddVelocity(deltaTime * float2(0, -9.8f));

        _bodies[i]->SetForce(float2(0, 0));
    }

    // clean up, restore the members
    scene->m_integrator = originalIntegrator;
    scene->m_deltaTime = originalDeltaTime;
}
