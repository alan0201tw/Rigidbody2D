#include "obb.hpp"

#include "manifold.hpp"
#include "circle.hpp"
#include "collision.hpp"
#include "util.hpp"

#include <vector>
#include <algorithm>
#include <iostream>

#include "GL/freeglut.h"

OBB::float2 OBB::GetSupportPoint(const float2& dir) const
{
    // init as max
    float bestProjection = -1e9f;
    float2 bestVertex = float2(0.0f, 0.0f);

    std::array<float2, 4> vertices = GetLocalSpaceVertices();

    for(size_t i = 0; i < GetVertexCount(); ++i)
    {
        float2 v = vertices[i];
        float projection = linalg::dot( v, dir );
    
        if(projection > bestProjection)
        {
            bestVertex = v;
            bestProjection = projection;
        }
    }

    return bestVertex;
}

Manifold OBB::accept(const std::shared_ptr<const ShapeVisitor<Manifold>>& visitor) const
{
    return visitor->visitAABB(shared_from_this());
}

//////////////

float OBB::FindAxisLeastPenetration(
    size_t& faceIndexPtr, 
    const OBB& A, 
    const OBB& B)
{
    float bestDistance = -1e9f;
    size_t bestIndex = 0u;

    std::array<float2, 4> vertices = A.GetLocalSpaceVertices();
    std::array<float2, 4> normals = A.GetLocalSpaceNormals();

    float2x2 rotMatrixOfA = getRotationMatrix(A.m_body->GetOrientation());
    float2x2 rotMatrixOfB = getRotationMatrix(B.m_body->GetOrientation());

    for(size_t i = 0; i < A.GetVertexCount(); ++i)
    {
        // Retrieve a face normal from A
        float2 n = normals[i];
        float2 nw = linalg::mul(rotMatrixOfA, n);

        // Transform face normal into B's model space
        // Mat2 buT = B.u.Transpose( );
        float2x2 invRotMatrixOfB = linalg::transpose(rotMatrixOfB);
        n = linalg::mul(invRotMatrixOfB, nw);

        // Retrieve support point from B along -n
        float2 s = B.GetSupportPoint( -n );

        // Retrieve vertex on face from A, transform into
        // B's model space
        float2 v = vertices[i];
        v = linalg::mul(rotMatrixOfA, v) + A.m_body->GetPosition();
        v -= B.m_body->GetPosition();
        v = linalg::mul(invRotMatrixOfB, v);

        // Compute penetration distance (in B's model space)
        float d = linalg::dot( n, s - v );

        // Store greatest distance
        if(d > bestDistance)
        {
            bestDistance = d;
            bestIndex = i;
        }
    }
    
    faceIndexPtr = bestIndex;
    return bestDistance;
}

std::array<float2, 2> OBB::FindIncidentFace( 
    const OBB& RefPoly, 
    const OBB& IncPoly, 
    size_t referenceIndex)
{
    // const std::array<float2, 4> refVertices = RefPoly.GetLocalSpaceVertices();
    const std::array<float2, 4> refNormals = RefPoly.GetLocalSpaceNormals();
    const std::array<float2, 4> incVertices = IncPoly.GetLocalSpaceVertices();
    const std::array<float2, 4> incNormals = IncPoly.GetLocalSpaceNormals();

    float2 referenceNormal = refNormals[referenceIndex];

    const float2x2 rotMatrixOfRef = 
        getRotationMatrix(RefPoly.m_body->GetOrientation());
    const float2x2 rotMatrixOfInc = 
        getRotationMatrix(IncPoly.m_body->GetOrientation());
    const float2x2 invRotMatrixOfInc = 
        linalg::transpose(getRotationMatrix(IncPoly.m_body->GetOrientation()));

    // Calculate normal in incident's frame of reference
    referenceNormal = linalg::mul(rotMatrixOfRef, referenceNormal); // To world space
    referenceNormal = linalg::mul(invRotMatrixOfInc, referenceNormal); // To incident's model space

    // Find most anti-normal face on incident polygon
	size_t incidentFace = 0u;
    float minDot = 1e9f;
    for(size_t i = 0; i < IncPoly.GetVertexCount(); ++i)
    {
        float dot = linalg::dot(referenceNormal, incNormals[i]);
        if(dot < minDot)
        {
            minDot = dot;
            incidentFace = i;
        }
    }

    std::array<float2, 2> vertexPosArray;
    // Assign face vertices for incidentFace
    vertexPosArray[0] = 
        linalg::mul(rotMatrixOfInc, incVertices[incidentFace])
         + IncPoly.m_body->GetPosition();

    incidentFace = 
        incidentFace + 1 >= IncPoly.GetVertexCount() ? 
        0 : incidentFace + 1;

    vertexPosArray[1] = 
        linalg::mul(rotMatrixOfInc, incVertices[incidentFace])
         + IncPoly.m_body->GetPosition();

    return vertexPosArray;
}

size_t OBB::Clip(float2 normal, float clipped, std::array<float2, 2> face)
{
    size_t sp = 0u;
    float2 out[2] = {
        face[0],
        face[1]
    };

    // Retrieve distances from each endpoint to the line
    // d = ax + by - c
    float d1 = linalg::dot( normal, face[0] ) - clipped;
    float d2 = linalg::dot( normal, face[1] ) - clipped;

    // If negative (behind plane) clip
    if(d1 <= 0.0f) out[sp++] = face[0];
    if(d2 <= 0.0f) out[sp++] = face[1];
    
    // If the points are on different sides of the plane
    if(d1 * d2 < 0.0f) // less than to ignore -0.0f
    {
        // Push interesection point
        float alpha = d1 / (d1 - d2);
        out[sp] = face[0] + alpha * (face[1] - face[0]);
        ++sp;
    }

    // Assign our new converted values
    face[0] = out[0];
    face[1] = out[1];

    // assert( sp != 3 );
    if(sp == 3)
    {
        throw std::runtime_error(" OBB Assertion failed : sp == 3; ");
    }

    return sp;
}

//////////////

Manifold OBB::visitAABB(const std::shared_ptr<const OBB>& _shape) const
{
    //return Manifold(m_body, _shape->m_body, 0, {}, float2(0,0), 0, false);
    Manifold dummyManifold = Manifold(m_body, _shape->m_body, 0, {}, float2(0,0), 0, false);

    // Check for a separating axis with A's face planes
    size_t faceA = 0u;
    float penetrationA = FindAxisLeastPenetration(
        faceA, *this, *_shape);
    if(penetrationA >= 0.0f)
        return dummyManifold;

    // Check for a separating axis with B's face planes
    size_t faceB = 0u;
    float penetrationB = FindAxisLeastPenetration(
        faceB, *_shape, *this);
    if(penetrationB >= 0.0f)
        return dummyManifold;

    size_t referenceIndex = 0u;
    bool flip; // Always point from a to b

    std::shared_ptr<const OBB> RefPoly; // Reference
    std::shared_ptr<const OBB> IncPoly; // Incident

    // Determine which shape contains reference face
    if(biasGreaterThan( penetrationA, penetrationB ))
    {
        RefPoly = shared_from_this();
        IncPoly = _shape;
        referenceIndex = faceA;
        flip = false;
    }
    else
    {
        RefPoly = _shape;
        IncPoly = shared_from_this();
        referenceIndex = faceB;
        flip = true;
    }

    // World space incident face
    // float2 incidentFace[2];
    std::array<float2, 2> incidentFace = 
        FindIncidentFace( *RefPoly, *IncPoly, referenceIndex );

    const std::array<float2, 4> refVertices = RefPoly->GetLocalSpaceVertices();
    
    const float2x2 rotMatrixOfRef = 
        getRotationMatrix(RefPoly->m_body->GetOrientation());

    // Setup reference face vertices
    float2 v1 = refVertices[referenceIndex];
    referenceIndex = 
        (referenceIndex + 1 == RefPoly->GetVertexCount()) 
        ? 0 : referenceIndex + 1;
    float2 v2 = refVertices[referenceIndex];

    // Transform vertices to world space
    v1 = 
        linalg::mul(rotMatrixOfRef, v1) + RefPoly->m_body->GetPosition();
    v2 = 
        linalg::mul(rotMatrixOfRef, v2) + RefPoly->m_body->GetPosition();

    // Calculate reference face side normal in world space
    float2 sidePlaneNormal = (v2 - v1);
    sidePlaneNormal = safe_normalize(sidePlaneNormal);

    // Orthogonalize
    float2 refFaceNormal( sidePlaneNormal.y, -sidePlaneNormal.x );

    // ax + by = c
    // c is distance from origin
    float refC = linalg::dot( refFaceNormal, v1 );
    float negSide = -linalg::dot( sidePlaneNormal, v1 );
    float posSide =  linalg::dot( sidePlaneNormal, v2 );

    // Clip incident face to reference face side planes
    if(Clip( -sidePlaneNormal, negSide, incidentFace ) < 2)
        return dummyManifold; // Due to floating point error, possible to not have required points

    if(Clip(  sidePlaneNormal, posSide, incidentFace ) < 2)
        return dummyManifold; // Due to floating point error, possible to not have required points

    Manifold m = dummyManifold;

    // Flip
    m.m_normal = flip ? -refFaceNormal : refFaceNormal;

    // Keep points behind reference face
    int cp = 0; // clipped points behind reference face
    float separation = linalg::dot( refFaceNormal, incidentFace[0] ) - refC;
    if(separation <= 0.0f)
    {
        m.m_contactPoints[cp] = incidentFace[0];
        m.m_penetration = -separation;
        ++cp;
    }
    else
        m.m_penetration = 0;

    separation = linalg::dot( refFaceNormal, incidentFace[1] ) - refC;
    if(separation <= 0.0f)
    {
        m.m_contactPoints[cp] = incidentFace[1];

        m.m_penetration += -separation;
        ++cp;

        // Average penetration
        m.m_penetration /= (float)cp;
    }

    m.m_contactPointCount = cp;

    // data setup
    m.m_isHit = true;

    return m;
}

Manifold OBB::visitCircle(const std::shared_ptr<const Circle>& _shape) const
{
    auto manifold = CollisionHelper::GenerateManifold(
        shared_from_this(),
        _shape
    );

    return manifold;
}

void OBB::Render() const
{
    glPushMatrix();

    glTranslatef(m_body->GetPosition().x, m_body->GetPosition().y, 0);
    glRotatef(radianToDegree(m_body->GetOrientation()), 0, 0, 1);

    glBegin(GL_LINE_LOOP);
    {
        float2 half_extent = m_extent / 2.0f;

        glVertex2f(0 - half_extent[0], 0 - half_extent[1]);
        glVertex2f(0 - half_extent[0], 0 + half_extent[1]);
        glVertex2f(0 + half_extent[0], 0 + half_extent[1]);
        glVertex2f(0 + half_extent[0], 0 - half_extent[1]);
    }
    glEnd();

    glBegin(GL_POINTS);
    {
        glPushMatrix();

        glVertex2f(0, 0);

        glPopMatrix();
    }
    glEnd();

    glPopMatrix();
}