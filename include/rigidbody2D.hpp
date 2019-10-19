#pragma once

#include "linalg.h"

#include <memory>

class Shape;

class RigidBody2D
{
    typedef linalg::aliases::float2 float2;
private:
    float2 m_position;

private:
    float2 m_velocity;
    float2 m_force;

    float m_restitution;
    float m_mass;
	float m_invMass;

    float m_staticFriction;
    float m_dynamicFriction;

    // Angular components
    float m_orientation; // radians
    float m_angularVelocity;
    float m_torque;
    float m_inertia; // moment of inertia

    std::shared_ptr<Shape> m_shape;

public:
	RigidBody2D(
		std::shared_ptr<Shape> _shape, float2 _position, float _restitution,
		float _mass, float _staticFriction, float _dynamicFriction)
		: m_position(_position), m_velocity(float2(0, 0)), m_force(float2(0, 0))
		, m_restitution(_restitution), m_mass(_mass), m_invMass((m_mass == 0.0f) ? 0.0f : (1 / m_mass))
		, m_staticFriction(_staticFriction), m_dynamicFriction(_dynamicFriction)
		, m_orientation(0.0f), m_angularVelocity(0.0f), m_torque(0.0f), m_inertia(1.0f)
		, m_shape(_shape)
	{}


    inline std::shared_ptr<Shape> GetShape() { return m_shape; }
    inline float2 GetPosition() { return m_position; }
    inline float2 GetVelocity() { return m_velocity; }
	inline float2 GetForce() { return m_force; }
    inline float GetOrientation() { return m_orientation; }
	inline float GetAngularVelocity() { return m_angularVelocity; }
	inline float GetTorque() { return m_torque; }

	inline float GetMass() { return m_mass; }
	inline float GetInvMass() { return m_invMass; }

	inline float GetInertia() { return m_inertia; }

    // notice that we do not do negative mass testing here
    void SetMass(float _mass)
	{
		m_mass = _mass;
		m_invMass = (m_mass == 0.0f) ? 0.0f : (1 / m_mass);
	}
	void SetPosition(float2 _pos) { m_position = _pos; }
	void AddPosition(float2 _pos) { m_position += _pos; }

    void SetVelocity(float2 _velo) { m_velocity = _velo; }
    void AddVelocity(float2 _velo) { m_velocity += _velo; }

    void SetForce(float2 _force) { m_force = _force; }
    void AddForce(float2 _force) { m_force += _force; }

	void SetOrientation(float _ori) { m_orientation = _ori; }
	void AddOrientation(float _ori) { m_orientation += _ori; }

	void SetAngularVelocity(float _angVel) { m_angularVelocity = _angVel; }
	void AddAngularVelocity(float _angVel) { m_angularVelocity += _angVel; }

	friend class Manifold;
};