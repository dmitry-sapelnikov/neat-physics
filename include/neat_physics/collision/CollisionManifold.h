// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025-2026 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

// Includes
#include "neat_physics/collision/CollisionPoint.h"

namespace nph
{

/// Collision manifold between 2 bodies
/// \todo Should be turned into a class for encapsulation
template <uint16_t D>
struct CollisionManifold
{
	/// Body A
	const uint32_t bodyA;

	/// Body B
	const uint32_t bodyB;

	/// Collision points
	CollisionPointArray<D> points;

	/// The actual number of collision points
	uint32_t pointsCount;

	/// Default constructor (no initialization)
	CollisionManifold() noexcept = default;

	/// Constructor
	/// Asserts that bodyIndA < bodyIndB
	CollisionManifold(
		uint32_t inBodyIndA,
		uint32_t inBodyIndB) noexcept :

		bodyA(inBodyIndA),
		bodyB(inBodyIndB),
		pointsCount(0)
	{
		assert(bodyA < bodyB);
	}
};

} // namespace nph
