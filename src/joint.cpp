#include "joint.hpp"

#include "GL/freeglut.h"

#include "rigidbody2D.hpp"
#include "util.hpp"

#include <iostream>

void SpringJoint::ApplyConstriant() const
{
    float distance = safe_distance(m_body0->GetPosition(), m_body1->GetPosition());

    auto diffPos_unit = safe_normalize(m_body1->GetPosition() - m_body0->GetPosition());
    // -Ks * deltaL * diffPos_unit
    float2 vec0to1 = m_stiffness * (distance - m_restLength) * diffPos_unit;

    auto deltaVel = m_body1->GetVelocity() - m_body0->GetVelocity();
    float2 damper = (1.0f / 30.0f) * m_stiffness * linalg::dot(deltaVel, diffPos_unit) * diffPos_unit;

    m_body0->AddForce( vec0to1 + damper );
    m_body1->AddForce( (vec0to1 + damper) * -1 );
}

void SpringJoint::Render() const
{
    glPushMatrix();
    glPushAttrib(GL_CURRENT_BIT);
    {
        glBegin(GL_LINES);
        // red for spring joint
        glColor3f(1, 0, 0);
        glVertex2f( m_body0->GetPosition().x, m_body0->GetPosition().y );
        glVertex2f( m_body1->GetPosition().x, m_body1->GetPosition().y );
        glEnd();
    }
    glPopAttrib();
    glPopMatrix();
}

void DistanceJoint::ApplyConstriant() const
{
    // Reference :
    // https://wildbunny.co.uk/blog/2011/04/06/physics-engines-for-dummies/


    float distance = safe_distance(m_body0->GetPosition(), m_body1->GetPosition());
    // distance joint is for maintaining the distance difference of two
    // bodies lesser than a length, so ignore it if the length is already lesser
    if(distance <= m_restLength)
        return;
    // if the two bodies are getting toward each other, ignore this constraint
    // if(linalg::dot(m_body0->GetVelocity(), m_body1->GetVelocity()) < 0.0f)
    //     return;

    // if both object is static, return
    if(m_body0->GetMass() == 0.0f && m_body1->GetMass() == 0.0f)
        return;

    const float2 axis = m_body1->GetPosition() - m_body0->GetPosition();
    const float2 unit_axis = safe_normalize(axis);

    const float relVel = 
        linalg::dot(m_body1->GetVelocity() - m_body0->GetVelocity(), unit_axis);
    const float relDist = distance - m_restLength;

	// 0.001f need to be larger than deltaTime
    const float remove = relVel + relDist / m_deltaTime;
    const float impulse_scalar = remove / (m_body0->GetInvMass() + m_body1->GetInvMass());

    const float2 impulse = impulse_scalar * unit_axis;

    m_body0->AddVelocity(impulse * m_body0->GetInvMass());
    m_body1->AddVelocity(-1 * impulse * m_body1->GetInvMass());
}

void DistanceJoint::Render() const
{
    glPushMatrix();
    glPushAttrib(GL_CURRENT_BIT);
    {
        glBegin(GL_LINES);
        // green for distance joint
        glColor3f(0, 1, 0);
        glVertex2f( m_body0->GetPosition().x, m_body0->GetPosition().y );
        glVertex2f( m_body1->GetPosition().x, m_body1->GetPosition().y );
        glEnd();
    }
    glPopAttrib();
    glPopMatrix();
}