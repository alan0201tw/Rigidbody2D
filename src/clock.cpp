#include "clock.hpp"

Clock::m_clock::time_point Clock::m_origin;

void Clock::Reset()
{
    m_origin = m_clock::now();
}

double Clock::Elapsed()
{
    auto current = m_clock::now();
    return std::chrono::duration_cast<m_tickUnit>(current - m_origin).count() / 
        static_cast<double>(std::chrono::duration_cast<m_tickUnit>(std::chrono::seconds(1)).count());
}