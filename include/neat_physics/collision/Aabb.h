// A minimalistic 2D physics engine (https://github.com/dmitry-sapelnikov/neat-physics)
// SPDX-FileCopyrightText: 2025 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

// Includes
#include "neat_physics/math/Vec2.h"

namespace nph
{

/// 2D axis-aligned bounding box, immutable for simplicity
struct Aabb
{
	/// Minimum corner
	const Vec2 min;

	/// Maximum corner
	const Vec2 max;

	/// Default constructor (no initialization)
	Aabb() noexcept = default;

	/// Min-max point constructor; asserts that min <= max
	Aabb(
		const Vec2& inMin,
		const Vec2& inMax) noexcept :
		min(inMin),
		max(inMax)
	{
		assert(min.x <= max.x);
		assert(min.y <= max.y);
	}
};

// namespace nph
}
