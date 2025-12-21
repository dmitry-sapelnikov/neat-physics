// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
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
template <uint16_t D>
class ContactManifold
{
public:
	/// Constructor
	ContactManifold(
		Body<D>& bodyA,
		Body<D>& bodyB,
		const CollisionManifold<D>& manifold) noexcept;

	/// Returns the first body
	const Body<D>& getBodyA() const noexcept
	{
		return *mBodyA;
	}

	/// Returns the second body
	const Body<D>& getBodyB() const noexcept
	{
		return *mBodyB;
	}

	/// Returns the contact count
	inline [[nodiscard]] uint32_t getContactCount() const noexcept
	{
		return mContactCount;
	}

	/// Returns the contact at given index
	inline const ContactPoint<D>& getContact(uint32_t index) const noexcept
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
	void update(const CollisionManifold<D>& newManifold) noexcept;

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
	Body<D>* mBodyA;

	/// Second body
	Body<D>* mBodyB;

	/// Contact array
	std::array<ContactPoint<D>, CollisionPoint<D>::MAX_POINTS> mContacts;

	/// Actual contact count
	uint32_t mContactCount;

	/// Obsoletion flag
	bool mObsolete;

	/// Contact pair friction coefficient
	float mFriction;
};

template <uint16_t D>
ContactManifold<D>::ContactManifold(
	Body<D>& bodyA,
	Body<D>& bodyB,
	const CollisionManifold<D>& manifold) noexcept :

	mBodyA(&bodyA),
	mBodyB(&bodyB),
	mContacts{ manifold.points[0], manifold.points[1] },
	mContactCount(manifold.pointsCount),
	mObsolete(false),

	// A well-known approximation for friction between two materials
	// \todo: introduce material pairs
	mFriction(std::sqrt(mBodyA->friction* mBodyB->friction))
{
	assert(0 < mContactCount && mContactCount <= MAX_COLLISION_POINTS);
}

template <uint16_t D>
void ContactManifold<D>::update(
	const CollisionManifold<D>& newManifold) noexcept
{
	// Make a backup of old contacts
	std::array<ContactPoint<D>, CollisionPoint<D>::MAX_POINTS> oldContacts;
	const uint32_t oldCount = mContactCount;
	for (uint32_t i = 0; i < mContactCount; ++i)
	{
		oldContacts[i] = mContacts[i];
	}

	for (uint32_t i = 0; i < newManifold.pointsCount; ++i)
	{
		mContacts[i] = ContactPoint<D>(newManifold.points[i]);
		for (ContactPoint<D>*oldContact = oldContacts.data();
			oldContact < oldContacts.data() + oldCount;
			++oldContact)
		{
			if (newManifold.points[i].featurePair ==
				oldContact->getPoint().featurePair)
			{
				mContacts[i].updateFrom(*oldContact);
				break;
			}
		}
	}
	mContactCount = newManifold.pointsCount;
	mObsolete = false;
}

template <uint16_t D>
void ContactManifold<D>::prepareToSolve() noexcept
{
	for (ContactPoint<D>*contact = mContacts.data();
		contact < mContacts.data() + mContactCount;
		++contact)
	{
		contact->prepareToSolve(*mBodyA, *mBodyB);
	}
}

template <uint16_t D>
void ContactManifold<D>::solveVelocities() noexcept
{
	for (ContactPoint<D>*contact = mContacts.data();
		contact < mContacts.data() + mContactCount;
		++contact)
	{
		contact->solveVelocities(*mBodyA, *mBodyB, mFriction);
	}
}

template <uint16_t D>
void ContactManifold<D>::solvePositions() noexcept
{
	for (ContactPoint<D>*contact = mContacts.data();
		contact < mContacts.data() + mContactCount;
		++contact)
	{
		contact->solvePositions(*mBodyA, *mBodyB);
	}
}

template <uint16_t D>
void ContactManifold<D>::onBodiesReallocation(
	std::ptrdiff_t memoryOffsetInBytes) noexcept
{
	mBodyA = reinterpret_cast<Body<D>*>(
		reinterpret_cast<std::byte*>(mBodyA) + memoryOffsetInBytes);

	mBodyB = reinterpret_cast<Body<D>*>(
		reinterpret_cast<std::byte*>(mBodyB) + memoryOffsetInBytes);
}

}
