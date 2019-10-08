#pragma once

#include <chrono>

class Clock
{
private:
    typedef std::chrono::high_resolution_clock m_clock;
    typedef std::chrono::nanoseconds m_tickUnit;

    static m_clock::time_point m_origin;

public:
    static void Reset();
    static double Elapsed();
};