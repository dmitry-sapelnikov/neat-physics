// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

// Includes
#include "neat_physics/math/Vec2.h"

namespace nph
{

/// Represents a 2D plane defined by a normal and an offset
template <uint64_t D>
struct Plane
{
	/// Normal (asserted to be normalized)
	const Vec<D> normal;

	/// Offset from the origin
	float offset;

	/// Default constructor (no initialization)
	Plane() noexcept = default;

	/// Constructs plane from normal and offset
	Plane(
		const Vec<D>& inNormal,
		float inOffset) noexcept :

		normal(inNormal),
		offset(inOffset)
	{
		assert(normal.isNormalized());
	}

	/// Constructs plane from normal and a point on the plane
	Plane(
		const Vec<D>& inNormal,
		const Vec<D>& inOrigin) noexcept :

		Plane(inNormal, dot(inNormal, inOrigin))
	{
	}

	/// Constructs plane from normal, a point on the plane + extra offset
	Plane(
		const Vec<D>& inNormal,
		const Vec<D>& inOrigin,
		float inOffset) noexcept :

		Plane(inNormal, dot(inNormal, inOrigin) + inOffset)
	{
	}

	/// Returns the signed distance from the plane to the point
	[[nodiscard]] float getDistance(const Vec<D>& point) const noexcept
	{
		return dot(normal, point) - offset;
	}
};

/// 2D plane alias
using Plane2 = Plane<2>;

} // namespace nph
