// A minimalistic 2D physics engine (https://github.com/dmitry-sapelnikov/neat-physics)
// SPDX-FileCopyrightText: 2025 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

// Includes
#include "neat_physics/collision/CollisionSystem.h"
#include "NarrowPhase.h"

namespace nph
{

void CollisionSystem::update(CollisionCallback& callback)
{
	mCallback = &callback;
	mBroadPhase.update(*this);
	mCallback = nullptr;
}

void CollisionSystem::onCollision(uint32_t bodyIndA, uint32_t bodyIndB)
{
	const Body& bodyA = mBodies[bodyIndA];
	const Body& bodyB = mBodies[bodyIndB];

	CollisionManifold manifold(bodyIndA, bodyIndB);
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

// End of namespace nph
}
