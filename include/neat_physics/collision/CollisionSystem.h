// A minimalistic 2D physics engine (https://github.com/dmitry-sapelnikov/neat-physics)
// SPDX-FileCopyrightText: 2025 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

// Includes
#include <functional>
#include "neat_physics/collision/BroadPhase.h"
#include "neat_physics/collision/CollisionCallback.h"

namespace nph
{

/// Collision system, computes contact manifolds between geometries
class CollisionSystem
{
public:
	/// Constructor
	CollisionSystem(const BodyArray& bodies) noexcept :
		mBodies(bodies),
		mBroadPhase(bodies)
	{
	}

	/// Returns the broad-phase collision detector
	[[nodiscard]] const BroadPhase& getBroadPhase() const noexcept
	{
		return mBroadPhase;
	}

	/// Updates the collision manifolds
	template <typename CollisionCallback>
	void update(CollisionCallback callback)
	{
		mBroadPhase.update([this, callback](
			uint32_t bodyIndA,
			uint32_t bodyIndB)
			{
				const CollisionManifold manifold =
					getManifold(bodyIndA, bodyIndB);
				if (manifold.pointsCount > 0)
				{
					callback(manifold);
				}
			});
	}

private:
	CollisionManifold getManifold(uint32_t bodyIndA, uint32_t bodyIndB) const;

	/// Reference to the bodies
	const BodyArray& mBodies;

	/// Broad-phase collision detector
	BroadPhase mBroadPhase;

	/// A temporary pointer to the collision callback
	CollisionCallback* mCallback{ nullptr };
};

} // namespace nph
