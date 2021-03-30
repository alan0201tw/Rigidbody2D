#pragma once

#include <vector>

#include "rigidbody2D.hpp"

class Scene;

class Integrator
{
protected:
    typedef linalg::aliases::float2 float2;
    typedef std::shared_ptr<RigidBody2D> BodyRef;

public:
	virtual void Integrate(Scene& scene) = 0;
};

class ExplicitEulerIntegrator : public Integrator
{
public:
    virtual void Integrate(Scene& scene) override;
};

class SymplecticEulerIntegrator : public Integrator
{
public:
	virtual void Integrate(Scene& scene) override;
};

class NewtonIntegrator : public Integrator
{
public:
    virtual void Integrate(Scene& scene) override;
};

class RungeKuttaFourthIntegrator : public Integrator
{
private:
    struct StateStep
    {
        float2 position;
        float2 velocity;

		float orientation;
		float angularVelocity;
    };

public:
    virtual void Integrate(Scene& scene) override;
};