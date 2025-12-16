#pragma once

// Includes
#include "neat_physics/collision/CollisionPoint.h"

namespace nph
{

/// Contact manifold between 2 bodies
/// \todo Should be turned into a class for encapsulation
struct CollisionManifold
{
	/// Index of body A
	const uint32_t bodyIndA;

	/// Index of body B
	const uint32_t bodyIndB;

	/// Collision points
	CollisionPointArray points;

	/// The actual number of collision points
	uint32_t pointsCount;

	/// Default constructor (no initialization)
	CollisionManifold() noexcept = default;

	/// Constructor
	/// Asserts that bodyIndA < bodyIndB
	CollisionManifold(
		uint32_t inBodyIndA,
		uint32_t inBodyIndB) noexcept :

		bodyIndA(inBodyIndA),
		bodyIndB(inBodyIndB),
		pointsCount(0)
	{
		assert(bodyIndA < bodyIndB);
	}
};

} // namespace nph
