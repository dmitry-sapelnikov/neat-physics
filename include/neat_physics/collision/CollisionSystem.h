#pragma once

// Includes
#include "neat_physics/collision/CollisionManifold.h"
#include "neat_physics/collision/BroadPhase.h"

namespace nph
{

/// Collision system, computes contact manifolds between geometries
class CollisionSystem
{
public:
	/// Collision manifold container type
	using CollisionManifoldArray = std::vector<CollisionManifold>;

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

	/// Returns the collision manifolds computed in the last update
	[[nodiscard]] const CollisionManifoldArray& getCollisionManifolds() noexcept
	{
		return mManifolds;
	}

	/// Updates the collision manifolds
	void update();

private:
	/// Reference to the bodies
	const BodyArray& mBodies;

	/// Broad-phase collision detector
	BroadPhase mBroadPhase;

	/// Contact manifolds
	CollisionManifoldArray mManifolds;
};

} // namespace nph
