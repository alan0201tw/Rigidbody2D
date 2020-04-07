#include "util.hpp"

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

float radianToDegree(float radian)
{
    return radian / acos(-1.0f) * 180.0f;
}

float2x2 getRotationMatrix(float radian)
{
    const float c = std::cos(radian);
    const float s = std::sin(radian);

    float2x2 result;
    result[0][0] = c;
    result[0][1] = -s;
    result[1][0] = s;
    result[1][1] = c;

    return result;
}