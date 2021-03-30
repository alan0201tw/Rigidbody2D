#pragma once

#include "linalg.h"

#include "shape.hpp"

#include <vector>

class OBB : public Shape, public std::enable_shared_from_this<OBB>
{
    typedef linalg::aliases::float2 float2;
    typedef linalg::aliases::float2x2 float2x2;
private:
    float2 m_extent;

    float2 GetSupportPoint(const float2& dir) const;

    // helper functions for helping collision detection and manifold generation

    static float FindAxisLeastPenetration(
        size_t& faceIndexPtr, 
        const OBB& A, 
        const OBB& B);
    
    static std::array<float2, 2> FindIncidentFace( 
        const OBB& RefPoly, 
        const OBB& IncPoly, 
        size_t referenceIndex);

    static size_t Clip(float2 normal, float clipped, std::array<float2, 2> face);

public:
    OBB(float2 _extent) : m_extent(_extent) {}

    virtual Manifold accept(const std::shared_ptr<const ShapeVisitor<Manifold>>& visitor) const override;

    virtual Manifold visitAABB(const std::shared_ptr<const OBB>& _shape) const override;
    virtual Manifold visitCircle(const std::shared_ptr<const Circle>& _shape) const override;

    virtual void Render() const override;

    inline size_t GetVertexCount() const { return 4u; }

    inline std::array<float2, 4> GetLocalSpaceVertices() const
    {
        const float2 half_extent = m_extent / 2.0f;
        std::array<float2, 4> vertices = 
        {
            float2( half_extent.x, -half_extent.y),
            float2( half_extent.x,  half_extent.y),
            float2(-half_extent.x,  half_extent.y),
            float2(-half_extent.x, -half_extent.y),
        };

        return vertices;
    }

    inline std::array<float2, 4> GetLocalSpaceNormals() const
    {
        // the order of these normals should match the order of local space vertices
        std::array<float2, 4> normals = 
        {
            float2( 1,  0),
            float2( 0,  1),
            float2(-1,  0),
            float2( 0, -1)
        };

        return normals;
    }

    friend class CollisionHelper;
};