// A minimalistic 2D physics engine (https://github.com/dmitry-sapelnikov/neat-physics)
// SPDX-FileCopyrightText: 2025 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

// Includes
#include <unordered_map>
#include "neat_physics/collision/CollisionCallback.h"
#include "neat_physics/dynamics/ContactManifold.h"

namespace nph
{

/// Solver for contact constraints between bodies
class ContactSolver : public CollisionCallback
{
public:
	/// Map of contact pairs.
	/// The key is combination of two body IDs,
	/// the value is the index in the manifold array.
	using ContactPairsMap = std::unordered_map<uint64_t, uint32_t>;

	/// Entry in the manifolds array
	/// The first element is a pointer to the contact pair map entry
	using ManifoldsArrayEntry =
		std::pair<ContactPairsMap::iterator, ContactManifold>;

	/// Array of contact manifolds
	using ManifoldsArray = std::vector<ManifoldsArrayEntry>;

	/// Constructor
	ContactSolver(BodyArray<2>& bodies) noexcept;

	/// Clears all contact manifolds
	void clear() noexcept;

	/// Returns the contact manifolds
	[[nodiscard]] const ManifoldsArray& getManifolds() const noexcept
	{
		return mManifolds;
	}

	/// Prepares the contact manifolds update
	void prepareManifoldsUpdate() noexcept;

	/// Collision callback
	void onCollision(const CollisionManifold& collisionManifold);

	/// Finishes the contact manifolds update
	void finishManifoldsUpdate();

	/// Prepares the contact solver for velocity solving
	void prepareToSolve() noexcept;

	/// Solves the contact velocities
	void solveVelocities(uint32_t velocityIterations) noexcept;

	/// Solves the contact positions (penetration)
	void solvePositions(uint32_t positionIterations) noexcept;

	/// Called when bodies are reallocated
	/// \param memoryOffset the offset in BYTES between the previously allocated
	/// and newly allocated body arrays
	void onBodiesReallocation(std::ptrdiff_t memoryOffsetInBytes) noexcept;

private:
	/// Reference to the body array
	BodyArray<2>& mBodies;

	/// Persistent contact manifolds
	ContactPairsMap mContactPairs;

	/// Contact manifolds
	ManifoldsArray mManifolds;
};

}
