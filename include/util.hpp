#pragma once

#include "linalg.h"

using linalg::aliases::float2;

float2 safe_normalize(float2 a)
{
    float length_a = linalg::length(a);
    return (length_a == 0.0f) ? float2(0.0f, 0.0f) : a / length_a;
}

float safe_distance(float2 a, float2 b)
{
    float length_ab = linalg::length(a - b);
    return (length_ab == 0.0f) ? 0.0f : length_ab;
}