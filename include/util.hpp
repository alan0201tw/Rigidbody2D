#pragma once

#include "linalg.h"

using linalg::aliases::float2;
using linalg::aliases::float2x2;

float2 safe_normalize(float2 a);
float safe_distance(float2 a, float2 b);
float radianToDegree(float radian);
float2x2 getRotationMatrix(float radian);