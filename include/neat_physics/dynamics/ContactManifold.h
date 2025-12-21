// A minimalistic 2D physics engine (https://github.com/dmitry-sapelnikov/neat-physics)
// SPDX-FileCopyrightText: 2025 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

// Includes
#include "neat_physics/collision/CollisionManifold.h"
#include "neat_physics/dynamics/ContactPoint.h"

namespace nph
{

/// Persistent contact manifold between two bodies
/// Exploits temporal coherence to improve precision
class ContactManifold
{
public:
	/// Constructor
	ContactManifold(
		Body<2>& bodyA,
		Body<2>& bodyB,
		const CollisionManifold& manifold) noexcept;

	/// Returns the first body
	const Body<2>& getBodyA() const noexcept
	{
		return *mBodyA;
	}

	/// Returns the second body
	const Body<2>& getBodyB() const noexcept
	{
		return *mBodyB;
	}

	/// Returns the contact count
	inline [[nodiscard]] uint32_t getContactCount() const noexcept
	{
		return mContactCount;
	}

	/// Returns the contact at given index
	inline const ContactPoint& getContact(uint32_t index) const noexcept
	{
		assert(index < mContactCount);
		return mContacts[index];
	}

	/// Returns if the manifold is obsolete
	[[nodiscard]] bool isObsolete() const noexcept
	{
		return mObsolete;
	}

	/// Marks the manifold as obsolete
	void markObsolete() noexcept
	{
		mObsolete = true;
	}

	/// Updates the contact manifold with new contacts
	/// preserving impulses for matching contact points
	void update(const CollisionManifold& newManifold) noexcept;

	/// Prepares the contact manifold for velocity solving
	void prepareToSolve() noexcept;

	/// Solves the contact velocities
	void solveVelocities() noexcept;

	/// Solves the contact positions (penetration)
	void solvePositions() noexcept;

	/// Called when bodies are reallocated
	/// \param memoryOffset the offset in BYTES between the previously allocated
	/// and newly allocated body arrays
	void onBodiesReallocation(
		std::ptrdiff_t memoryOffset) noexcept;

private:
	/// First body
	Body<2>* mBodyA;

	/// Second body
	Body<2>* mBodyB;

	/// Contact array
	std::array<ContactPoint, MAX_COLLISION_POINTS> mContacts;

	/// Actual contact count
	uint32_t mContactCount;

	/// Obsoletion flag
	bool mObsolete;

	/// Contact pair friction coefficient
	float mFriction;
};

}
