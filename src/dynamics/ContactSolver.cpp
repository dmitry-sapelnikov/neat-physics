// A minimalistic 2D physics engine (https://github.com/dmitry-sapelnikov/neat-physics)
// SPDX-FileCopyrightText: 2025 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

// Includes
#include "neat_physics/dynamics/ContactSolver.h"

namespace nph
{

ContactSolver::ContactSolver(BodyArray& bodies) noexcept :
	mBodies(bodies)
{
}

void ContactSolver::clear() noexcept
{
	mManifolds.clear();
}

void ContactSolver::prepareToSolve() noexcept
{
	for (auto& [key, manifold] : mManifolds)
	{
		manifold.prepareToSolve();
	}
}

void ContactSolver::solveVelocities(uint32_t velocityIterations) noexcept
{
	for (uint32_t i = 0; i < velocityIterations; ++i)
	{
		for (auto& [key, manifold] : mManifolds)
		{
			manifold.solveVelocities();
		}
	}
}

void ContactSolver::solvePositions(uint32_t positionIterations) noexcept
{
	for (uint32_t i = 0; i < positionIterations; ++i)
	{
		for (auto& [key, manifold] : mManifolds)
		{
			manifold.solvePositions();
		}
	}
}

void ContactSolver::prepareManifoldsUpdate() noexcept
{
	for (auto& pair : mManifolds)
	{
		pair.second.markObsolete();
	}
}

void ContactSolver::onCollision(const CollisionManifold& manifold)
{
	Body* start = mBodies.data();
	Body* bodyA = start + manifold.bodyIndA;
	Body* bodyB = start + manifold.bodyIndB;

	const uint64_t key =
		(static_cast<uint64_t>(manifold.bodyIndA) << 32) |
		manifold.bodyIndB;

	if (auto iter = mManifolds.find(key);
		iter != mManifolds.end())
	{
		iter->second.update(manifold);
	}
	else
	{
		mManifolds.try_emplace(key, bodyA, bodyB, manifold);
	}
}

void ContactSolver::finishManifoldsUpdate()
{
	// Remove obsolete manifolds
	std::erase_if(
		mManifolds,
		[](const auto& pair)
		{
			return pair.second.isObsolete();
		});
}

} // namespace nph
