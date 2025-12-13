#pragma once

// Includes
#include <iostream>

// Global macros
/// Macro to suppress unused variable warnings
#define UNUSED(x) (void)(x)

namespace nph
{

// Global functions
/// Logs an error message, ensuring no exceptions are thrown
template <typename... Args>
void logError(Args... args) noexcept
{
    try
    {
        (std::cerr << ... << args) << std::endl;
    }
    catch (...)
    {
        // Avoid throwing while handling another exception
        // Probably we can just let the call of terminate()
    }
}

// End of nph namespace
}
