#pragma once

#include "linalg.h"

using linalg::aliases::float2;
using linalg::aliases::float2x2;

float2 safe_normalize(float2 a);
float safe_distance(float2 a, float2 b);
float radianToDegree(float radian);
float2x2 getRotationMatrix(float radian);

static inline bool biasGreaterThan(float a, float b)
{
    const float k_biasRelative = 0.95f;
    const float k_biasAbsolute = 0.01f;
    return a >= b * k_biasRelative + a * k_biasAbsolute;
}