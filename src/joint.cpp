#include "joint.hpp"

#include "GL/freeglut.h"

#include "rigidbody2D.hpp"

#include <iostream>

void SpringJoint::ApplyConstriant() const
{
    float distance = linalg::distance(m_body0->GetPosition(), m_body1->GetPosition());
    // if(distance <= m_restLength)
    //     return;
    // if the two objects are getting toward each other, do not resolve this
    // if(linalg::dot(m_body0->GetVelocity(), m_body1->GetVelocity()) < 0.0f)
    //     return;
    auto diffPos_unit = linalg::normalize(m_body1->GetPosition() - m_body0->GetPosition());
    // -Ks * deltaL * diffPos_unit
    float2 vec0to1 = m_stiffness * (distance - m_restLength) * diffPos_unit;

    auto deltaVel = m_body1->GetVelocity() - m_body0->GetVelocity();
    float2 damper = 0.1f * m_stiffness * linalg::dot(deltaVel, diffPos_unit) * diffPos_unit;

    m_body0->AddForce( vec0to1 + damper );
    m_body1->AddForce( (vec0to1 + damper) * -1 );
}

void SpringJoint::Render() const
{
    glPushMatrix();
    glPushAttrib(GL_CURRENT_BIT);
    {
        glBegin(GL_LINES);
        glColor3f(1, 0, 0);
        glVertex2f( m_body0->GetPosition().x, m_body0->GetPosition().y );
        glVertex2f( m_body1->GetPosition().x, m_body1->GetPosition().y );
        glEnd();
    }
    glPopAttrib();
    glPopMatrix();
}