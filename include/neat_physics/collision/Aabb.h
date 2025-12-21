// A minimalistic 2D physics engine (https://github.com/dmitry-sapelnikov/neat-physics)
// SPDX-FileCopyrightText: 2025 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

// Includes
#include "neat_physics/math/Vec2.h"

namespace nph
{

/// 2D axis-aligned bounding box, immutable for simplicity
template <uint16_t D>
struct Aabb
{
	/// Minimum corner
	const Vec<D> min;

	/// Maximum corner
	const Vec<D> max;

	/// Default constructor (no initialization)
	Aabb() noexcept = default;

	/// Min-max point constructor; asserts that min <= max
	Aabb(
		const Vec2& inMin,
		const Vec2& inMax) noexcept :
		min(inMin),
		max(inMax)
	{
		for (uint16_t i = 0; i < D; ++i)
		{
			assert(min[i] <= max[i]);
		}
	}
};

/// 2D AABB alias
using Aabb2 = Aabb<2>;


/// Computes the AABB for a box-shaped geometry
template <uint16_t D>
Aabb<D> getAabb(
	const Vec<D>& position,
	const Mat<D, D>& rotation,
	const Vec<D>& halfSize)
{
	const Mat<D, D> absRotation = abs(rotation);
	const Vec<D> halfExtents =
		halfSize.x * absRotation.col1 +
		halfSize.y * absRotation.col2;

	return {
		position - halfExtents,
		position + halfExtents
	};
}	

// namespace nph
}
