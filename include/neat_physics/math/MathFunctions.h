// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

// Includes
#include <cstdint>

namespace nph
{

/// Pi constant
constexpr float PI = 3.14159265358979323846f;

/// Converts radians to degrees
inline float toDegrees(float radians) noexcept
{
	return 180.0f * (radians / PI);
}

/// Converts degrees to radians
inline float toRadians(float degrees) noexcept
{
	return PI * (degrees / 180.0f);
}

} // namespace nph
