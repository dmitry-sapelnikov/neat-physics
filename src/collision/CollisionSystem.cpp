// A minimalistic 2D physics engine (https://github.com/dmitry-sapelnikov/neat-physics)
// SPDX-FileCopyrightText: 2025 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

// Includes
#include "neat_physics/collision/CollisionSystem.h"
#include "NarrowPhase.h"

namespace nph
{

void CollisionSystem::update()
{
	mManifolds.clear();
	mBroadPhase.update();

	const Body* start = mBodies.data();
	for (const auto& [indA, indB] : mBroadPhase.getCollidingPairs())
	{
		const Body* bodyA = start + indA;
		const Body* bodyB = start + indB;

		const Vec2Array2 positions { bodyA->position, bodyB->position };
		const RotationArray2 rotations { bodyA->rotation, bodyB->rotation };
		const Vec2Array2 halfSizes { bodyA->halfSize, bodyB->halfSize };

		CollisionManifold manifold(indA, indB);
		manifold.pointsCount = getBoxBoxCollision(
			positions,
			rotations,
			halfSizes,
			manifold.points);

		if (manifold.pointsCount > 0)
		{
			mManifolds.push_back(manifold);
		}
	}
}

// End of namespace nph
}
