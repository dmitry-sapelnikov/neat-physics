// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
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
template <uint16_t D>
class ContactSolver : public CollisionCallback<D>
{
public:
	/// Map of contact pairs.
	/// The key is combination of two body IDs,
	/// the value is the index in the manifold array.
	using ContactPairsMap = std::unordered_map<uint64_t, uint32_t>;

	/// Entry in the manifolds array
	/// The first element is a pointer to the contact pair map entry
	using ManifoldsArrayEntry =
		std::pair<ContactPairsMap::iterator, ContactManifold<D>>;

	/// Array of contact manifolds
	using ManifoldsArray = std::vector<ManifoldsArrayEntry>;

	/// Constructor
	ContactSolver(BodyArray<D>& bodies) noexcept :
		mBodies(bodies)
	{
	}

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
	void onCollision(const CollisionManifold<D>& collisionManifold);

	/// Finishes the contact manifolds update
	void finishManifoldsUpdate();

	/// Prepares the contact solver for velocity solving
	void prepareToSolve() noexcept;

	/// Solves the contact velocities
	void solveVelocities(uint32_t velocityIterations) noexcept;

	/// Solves the contact positions (penetration)
	void solvePositions(uint32_t positionIterations) noexcept;

	/// Called when bodies are reallocated
	/// \param memoryOffsetInBytes the offset in bytes
	/// between the previously allocated
	/// and newly allocated body arrays
	void onBodiesReallocation(std::ptrdiff_t memoryOffsetInBytes) noexcept;

private:
	/// Reference to the body array
	BodyArray<D>& mBodies;

	/// Persistent contact manifolds
	ContactPairsMap mContactPairs;

	/// Contact manifolds
	ManifoldsArray mManifolds;
};

template <uint16_t D>
void ContactSolver<D>::clear() noexcept
{
	mContactPairs.clear();
	mManifolds.clear();
}

template <uint16_t D>
void ContactSolver<D>::onBodiesReallocation(
	std::ptrdiff_t memoryOffsetInBytes) noexcept
{
	for (auto& pair : mManifolds)
	{
		pair.second.onBodiesReallocation(memoryOffsetInBytes);
	}
}

template <uint16_t D>
void ContactSolver<D>::prepareToSolve() noexcept
{
	for (auto& pair : mManifolds)
	{
		pair.second.prepareToSolve();
	}
}

template <uint16_t D>
void ContactSolver<D>::solveVelocities(uint32_t velocityIterations) noexcept
{
	for (uint32_t i = 0; i < velocityIterations; ++i)
	{
		for (auto& pair : mManifolds)
		{
			pair.second.solveVelocities();
		}
	}
}

template <uint16_t D>
void ContactSolver<D>::solvePositions(uint32_t positionIterations) noexcept
{
	for (uint32_t i = 0; i < positionIterations; ++i)
	{
		for (auto& pair : mManifolds)
		{
			pair.second.solvePositions();
		}
	}
}

template <uint16_t D>
void ContactSolver<D>::prepareManifoldsUpdate() noexcept
{
	for (auto& pair : mManifolds)
	{
		pair.second.markObsolete();
	}
}

template <uint16_t D>
void ContactSolver<D>::onCollision(const CollisionManifold<D>& manifold)
{
	const uint64_t key =
		(static_cast<uint64_t>(manifold.bodyIndA) << 32) |
		manifold.bodyIndB;

	if (auto iter = mContactPairs.find(key);
		iter != mContactPairs.end())
	{
		mManifolds[iter->second].second.update(manifold);
	}
	else
	{
		size_t manifoldCount = mManifolds.size();
		assert(manifoldCount <= std::numeric_limits<uint32_t>::max());
		const uint32_t index = static_cast<uint32_t>(manifoldCount);
		mManifolds.emplace_back(
			std::piecewise_construct,
			std::forward_as_tuple(mContactPairs.try_emplace(key, index).first),
			std::forward_as_tuple(
				mBodies[manifold.bodyIndA],
				mBodies[manifold.bodyIndB],
				manifold)
		);
	}
}

template <uint16_t D>
void ContactSolver<D>::finishManifoldsUpdate()
{
	// Remove obsolete manifolds, rewire the contact pairs map
	size_t mi = 0;
	while (mi != mManifolds.size())
	{
		if (mManifolds[mi].second.isObsolete())
		{
			mContactPairs.erase(mManifolds[mi].first);
			// Swap and pop
			if (mi != mManifolds.size() - 1)
			{
				mManifolds[mi] = std::move(mManifolds.back());
				mManifolds[mi].first->second = static_cast<uint32_t>(mi);
			}
			mManifolds.pop_back();
		}
		else
		{
			++mi;
		}
	}
}

} // nph namespace
