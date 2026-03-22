// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025-2026 Dmitry Sapelnikov
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
	/// Body view layout
	using BodyLayout = ecs::Layout<
		ecs::TransformBlock<D>,
		ecs::MassPropertiesBlock<D>,
		ecs::HalfSizeBlock<D>>;

	/// Body view alias
	using Bodies = ecs::ConstantView<BodyLayout>;

	/// Broad-phase collision detector alias
	using BroadPhase = BroadPhase<D>;

	/// Collision callback alias
	using CollisionCallback = CollisionCallback<D>;

	/// Constructor
	/// \note \p settings must be a reference to a persistent object
	CollisionSystem(
		const Bodies& bodies,
		CollisionPairMap& collisionPairs,
		const CollisionSettings& settings) noexcept :

		mBodies(bodies),
		mBroadPhase(bodies, collisionPairs, settings),
		mSettings(settings)
	{
	}

	/// Clears the collision system
	void clear() noexcept
	{
		mBroadPhase.clear();
	}

	/// Returns the broad-phase collision detector
	[[nodiscard]] const BroadPhase& getBroadPhase() const noexcept
	{
		return mBroadPhase;
	}

	/// Updates the collision manifolds
	void update(CollisionCallback& callback);

private:
	void onCollision(
		uint32_t bodyA,
		uint32_t bodyB,
		uint32_t& broadPhasePairTag) override;

	/// Body component view
	Bodies mBodies;

	/// Reference to the collision settings
	const CollisionSettings& mSettings;

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
void CollisionSystem<D>::onCollision(
	uint32_t bodyA,
	uint32_t bodyB,
	uint32_t& broadPhasePairTag)
{
	assert(bodyA < bodyB);
	CollisionManifold<D> manifold(bodyA, bodyB);

	const auto transforms = mBodies.getComponent<ecs::TransformTag>();
	const auto halfSizes = mBodies.getComponent<ecs::HalfSizeTag>();

	manifold.pointsCount = getBoxBoxCollision<D>(
		{ transforms[bodyA].position, transforms[bodyB].position },
		{ transforms[bodyA].rotation, transforms[bodyB].rotation },
		{ halfSizes[bodyA], halfSizes[bodyB] },
		mSettings,
		manifold.points);

	if (manifold.pointsCount != 0)
	{
		mCallback->onCollision(manifold, broadPhasePairTag);
	}
	else
	{
		// Reset broad-phase pair value
		broadPhasePairTag = CollisionPairMap::NULL_VALUE;
	}
}

} // namespace nph
