// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025-2026 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

// Includes
#include <cstdint>

namespace nph
{

/// Pi constant
constexpr float PI = 3.14159265358979323846f;

/// Returns sign of the number
inline constexpr [[nodiscard]] int getSign(float value) noexcept
{
	return 1 - 2 * (value < 0.0f);
}

/// Converts radians to degrees
inline constexpr [[nodiscard]] float toDegrees(float radians) noexcept
{
	return 180.0f * (radians / PI);
}

/// Converts degrees to radians
inline constexpr [[nodiscard]] float toRadians(float degrees) noexcept
{
	return PI * (degrees / 180.0f);
}

/// Dot product for 1D vectors
inline constexpr [[nodiscard]] float dot(float a, float b) noexcept
{
	return a * b;
}

} // namespace nph
