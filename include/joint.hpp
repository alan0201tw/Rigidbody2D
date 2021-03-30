#pragma once

#include <memory>

#include "linalg.h"

#include "rigidbody2D.hpp"

// an interface for any joints to implement
// since some joints might not need two bodies, I will not
// declare them in the parent class
class Joint
{
protected:
    typedef linalg::aliases::float2 float2;
public:
    virtual void ApplyConstriant() const = 0;
    virtual void Render() const = 0;
};

class SpringJoint : public Joint
{
private:
    std::shared_ptr<RigidBody2D> m_body0, m_body1;
    float m_restLength;
    float m_stiffness;

public:
    explicit SpringJoint(
        const std::shared_ptr<RigidBody2D>& _body0, 
        const std::shared_ptr<RigidBody2D>& _body1, 
        float _restLength,
        float _stiffness)
        : m_body0(_body0), m_body1(_body1), m_restLength(_restLength)
        , m_stiffness(_stiffness)
        {}

    virtual void ApplyConstriant() const override;
    virtual void Render() const override;
};

class DistanceJoint : public Joint
{
private:
    std::shared_ptr<RigidBody2D> m_body0, m_body1;
    float m_restLength;
	float m_deltaTime;

public:
    explicit DistanceJoint(
        const std::shared_ptr<RigidBody2D>& _body0, 
        const std::shared_ptr<RigidBody2D>& _body1, 
        float _restLength, float _deltaTime)
        : m_body0(_body0), m_body1(_body1), m_restLength(_restLength), m_deltaTime(_deltaTime)
        {}
    
    virtual void ApplyConstriant() const override;
    virtual void Render() const override;
};