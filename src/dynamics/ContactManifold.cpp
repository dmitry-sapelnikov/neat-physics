// A minimalistic 2D physics engine (https://github.com/dmitry-sapelnikov/neat-physics)
// SPDX-FileCopyrightText: 2025 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

// Includes
#include "neat_physics/dynamics/ContactManifold.h"


namespace nph
{

// Global classes
ContactManifold::ContactManifold(
	Body<2>& bodyA,
	Body<2>& bodyB,
	const CollisionManifold<2>& manifold) noexcept :

	mBodyA(&bodyA),
	mBodyB(&bodyB),
	mContacts{ manifold.points[0], manifold.points[1] },
	mContactCount(manifold.pointsCount),
	mObsolete(false),

	// A well-known approximation for friction between two materials
	// \todo: introduce material pairs
	mFriction(std::sqrt(mBodyA->friction * mBodyB->friction))
{
	assert(0 < mContactCount && mContactCount <= MAX_COLLISION_POINTS);
}

void ContactManifold::update(const CollisionManifold<2>& newManifold) noexcept
{
	// Make a backup of old contacts
	std::array<ContactPoint<2>, CollisionPoint<2>::MAX_POINTS> oldContacts;
	const uint32_t oldCount = mContactCount;
	for (uint32_t i = 0; i < mContactCount; ++i)
	{
		oldContacts[i] = mContacts[i];
	}

	for (uint32_t i = 0; i < newManifold.pointsCount; ++i)
	{
		mContacts[i] = ContactPoint<2>(newManifold.points[i]);
		for (ContactPoint<2>* oldContact = oldContacts.data();
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

void ContactManifold::prepareToSolve() noexcept
{
	for (ContactPoint<2>* contact = mContacts.data();
		contact < mContacts.data() + mContactCount;
		++contact)
	{
		contact->prepareToSolve(*mBodyA, *mBodyB);
	}
}

void ContactManifold::solveVelocities() noexcept
{
	for (ContactPoint<2>* contact = mContacts.data();
		contact < mContacts.data() + mContactCount;
		++contact)
	{
		contact->solveVelocities(*mBodyA, *mBodyB, mFriction);
	}
}

void ContactManifold::solvePositions() noexcept
{
	for (ContactPoint<2>* contact = mContacts.data();
		contact < mContacts.data() + mContactCount;
		++contact)
	{
		contact->solvePositions(*mBodyA, *mBodyB);
	}
}

void ContactManifold::onBodiesReallocation(
	std::ptrdiff_t memoryOffsetInBytes) noexcept
{
	mBodyA = reinterpret_cast<Body<2>*>(
		reinterpret_cast<std::byte*>(mBodyA) + memoryOffsetInBytes);

	mBodyB = reinterpret_cast<Body<2>*>(
		reinterpret_cast<std::byte*>(mBodyB) + memoryOffsetInBytes);
}

} // namespace nph
