// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025-2026 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

// Includes
#include <cstdint>
#include "neat_physics/math/Vec2.h"
#include "neat_physics/math/Mat33.h"

namespace nph
{

/// Generic inertia trait
template <uint16_t D, typename T>
struct InertiaTrait;

/// 2D inertia trait specialization
template <typename T>
struct InertiaTrait<2, T>
{
	/// Inertia representation for 2D
	using Type = T;
};

/// 3D inertia trait specialization
template <typename T>
struct InertiaTrait<3, T>
{
	/// Inertia representation for 3D
	using Type = Mat<3, 3, T>;
};

/// Inertia definition via trait
template <uint16_t D, typename T = float>
using Inertia = typename InertiaTrait<D, T>::Type;

/// Returns the inverse 2D inertia given the inertia value
inline float getInvInertia(float inertia) noexcept
{
	return (inertia == 0.0f) ? 0.0f : 1.0f / inertia;
}

/// Returns the inverse 3D inertia given the inertia value
inline Mat33 getInvInertia(const Mat33& inertia) noexcept
{
	/// \todo check all diagonal elements / find another way to detect zero inertia
	return (inertia[0][0] == 0.0f) ?
		Mat33({ 0.f , 0.f, 0.f }, { 0.f, 0.f, 0.f }, { 0.f, 0.f, 0.f }) :
		inertia.getInverse();
}

/// Returns the moment of inertia for a 2D box shape
inline float getBoxInertia(const Vec2& size, float mass)
{
	return mass * size.lengthSquared() / 12.0f;
}

/// Returns the moment of inertia for a 3D box shape
inline Mat33 getBoxInertia(const Vec3& size, float mass)
{
	return {
		{ mass * (size.y * size.y + size.z * size.z) / 12.f, 0.f, 0.f },
		{ 0.f, mass * (size.x * size.x + size.z * size.z) / 12.f, 0.f },
		{ 0.f, 0.f, mass * (size.x * size.x + size.y * size.y) / 12.f }
	};
}

} // namespace nph
