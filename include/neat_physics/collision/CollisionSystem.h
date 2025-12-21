// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

// Includes
#include <functional>
#include "neat_physics/collision/BroadPhase.h"
#include "neat_physics/collision/NarrowPhase.h"
#include "neat_physics/collision/CollisionCallback.h"

namespace nph
{

/// Collision system, computes manifolds between geometries
template <uint16_t D>
class CollisionSystem : private BroadPhaseCallback
{
public:
	/// Body array alias
	using BodyArray = std::vector<Body<D>>;

	/// Broad-phase collision detector alias
	using BroadPhase = BroadPhase<D>;

	/// Collision callback alias
	using CollisionCallback = CollisionCallback<D>;

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

template <uint16_t D>
void CollisionSystem<D>::update(CollisionCallback& callback)
{
	mCallback = &callback;
	mBroadPhase.update(*this);
	mCallback = nullptr;
}

template <uint16_t D>
void CollisionSystem<D>::onCollision(uint32_t bodyIndA, uint32_t bodyIndB)
{
	const Body<D>& bodyA = mBodies[bodyIndA];
	const Body<D>& bodyB = mBodies[bodyIndB];

	CollisionManifold<D> manifold(bodyIndA, bodyIndB);
	manifold.pointsCount = getBoxBoxCollision(
		{ bodyA.position, bodyB.position },
		{ bodyA.rotation, bodyB.rotation },
		{ bodyA.halfSize, bodyB.halfSize },
		manifold.points);

	if (manifold.pointsCount > 0)
	{
		mCallback->onCollision(manifold);
	}
}

} // namespace nph
