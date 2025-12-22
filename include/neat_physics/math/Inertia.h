// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

// Includes
#include <cstdint>
#include "neat_physics/math/Vec2.h"
#include "neat_physics/math/Mat33.h"

namespace nph
{

/// Generic inertia trait
template <uint16_t D>
struct InertiaTrait;

/// 2D inertia trait specialization
template <>
struct InertiaTrait<2>
{
	using Type = float;
};

/// 3D inertia trait specialization
template <>
struct InertiaTrait<3>
{
	using Type = Mat33;
};

/// Inertia definition via trait
template <uint16_t D>
using Inertia = typename InertiaTrait<D>::Type;

/// Returns the inverse 2D inertia given the inertia value
inline float getInvInertia(float inertia) noexcept
{
	return (inertia == 0.0f) ? 0.0f : 1.0f / inertia;
}

/// Returns the moment of inertia for a 2D box shape
inline float getBoxInertia(const Vec2& size, float mass)
{
	return mass * size.lengthSquared() / 12.0f;
}

} // namespace nph
