#pragma once

/**
 *  This is where all the collision detection and manifold generation code
 *  lives. Classes that implements the shape visitor interface should be
 *  calling helper functions here, instead of implementing its own collision
 *  detection method.
 */

#include "obb.hpp"
#include "circle.hpp"

#include "manifold.hpp"

class CollisionHelper
{
public:
    // AABB to AABB
    static Manifold GenerateManifold(const std::shared_ptr<const OBB>& _a, const std::shared_ptr<const OBB>& _b);
    // AABB to Circle
    static Manifold GenerateManifold(const std::shared_ptr<const OBB>& _a, const std::shared_ptr<const Circle>& _b);
    // Circle to Circle
    static Manifold GenerateManifold(const std::shared_ptr<const Circle>& _a, const std::shared_ptr<const Circle>& _b);
};