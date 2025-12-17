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
class CollisionSystem : private BroadPhaseCallback
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
	void update(CollisionCallback& callback);

private:
	void onCollision(uint32_t bodyIndA, uint32_t bodyIndB) override;

	/// Reference to the bodies
	const BodyArray& mBodies;

	/// Broad-phase collision detector
	BroadPhase mBroadPhase;

	/// A temporary pointer to the collision callback
	CollisionCallback* mCallback{ nullptr };
};

} // namespace nph
