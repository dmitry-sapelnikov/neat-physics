// A minimalistic 2D physics engine (https://github.com/dmitry-sapelnikov/neat-physics)
// SPDX-FileCopyrightText: 2025 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

// Includes
#include "neat_physics/math/Vec2.h"

namespace nph
{

/// Represents a 2D plane defined by a normal and an offset
struct Plane
{
	/// Normal (asserted to be normalized)
	const Vec2 normal;

	/// Offset from the origin
	float offset;

	/// Default constructor (no initialization)
	Plane() noexcept = default;

	/// Construct plane from normal and offset
	Plane(
		const Vec2& inNormal,
		float inOffset) noexcept :

		normal(inNormal),
		offset(inOffset)
	{
		assert(normal.isNormalized());
	}

	/// Construct plane from normal and a point on the plane
	Plane(
		const Vec2& inNormal,
		const Vec2& inOrigin) noexcept :

		Plane(inNormal, dot(inNormal, inOrigin))
	{
	}

	/// Construct plane from normal, point on the plane + extra offset
	Plane(
		const Vec2& inNormal,
		const Vec2& inOrigin,
		float inOffset) noexcept :

		Plane(inNormal, dot(inNormal, inOrigin) + inOffset)
	{
	}

	/// Returns the signed distance from the plane to the point
	[[nodiscard]] float getDistance(const Vec2& point) const noexcept
	{
		return dot(normal, point) - offset;
	}
};

} // namespace nph
