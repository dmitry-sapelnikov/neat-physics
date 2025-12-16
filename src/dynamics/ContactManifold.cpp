// A minimalistic 2D physics engine (https://github.com/dmitry-sapelnikov/neat-physics)
// SPDX-FileCopyrightText: 2025 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

// Includes
#include "neat_physics/dynamics/ContactManifold.h"


namespace nph
{

// Global classes
ContactManifold::ContactManifold(
	Body* bodyA,
	Body* bodyB,
	const CollisionManifold& manifold) :

	mBodyA(bodyA),
	mBodyB(bodyB),
	mContacts{ manifold.points[0], manifold.points[1] },
	mContactCount(manifold.pointsCount),
	mObsolete(false),

	// A well-known approximation for friction between two materials
	// \todo: introduce material pairs
	mFriction(std::sqrt(mBodyA->friction * mBodyB->friction))
{
	assert(0 < mContactCount && mContactCount <= MAX_COLLISION_POINTS);
}

void ContactManifold::update(const CollisionManifold& newManifold)
{
	// Make a backup of old contacts
	ContactPoint oldContacts[MAX_COLLISION_POINTS];
	const uint32_t oldCount = mContactCount;
	for (uint32_t i = 0; i < mContactCount; ++i)
	{
		oldContacts[i] = mContacts[i];
	}

	for (uint32_t i = 0; i < newManifold.pointsCount; ++i)
	{
		mContacts[i] = ContactPoint(newManifold.points[i]);
		for (ContactPoint* oldContact = oldContacts;
			oldContact < oldContacts + oldCount;
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

void ContactManifold::prepareToSolve(float invTimeStep)
{
	for (ContactPoint* contact = mContacts.data();
		contact < mContacts.data() + mContactCount;
		++contact)
	{
		contact->prepareToSolve(*mBodyA, *mBodyB, invTimeStep);
	}
}

void ContactManifold::solveVelocities()
{
	for (ContactPoint* contact = mContacts.data();
		contact < mContacts.data() + mContactCount;
		++contact)
	{
		contact->solveVelocities(*mBodyA, *mBodyB, mFriction);
	}
}

void ContactManifold::solvePositions()
{
	for (ContactPoint* contact = mContacts.data();
		contact < mContacts.data() + mContactCount;
		++contact)
	{
		contact->solvePositions(*mBodyA, *mBodyB);
	}
}

} // namespace nph
