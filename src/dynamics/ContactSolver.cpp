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
	mContactPairs.clear();
	mManifolds.clear();
}

void ContactSolver::onBodiesReallocation(std::ptrdiff_t memoryOffsetInBytes) noexcept
{
	for (auto& pair : mManifolds)
	{
		pair.second.onBodiesReallocation(memoryOffsetInBytes);
	}
}

void ContactSolver::prepareToSolve(float timeStep) noexcept
{
	for (auto& [key, manifold] : mManifolds)
	const float invTimeStep = 1.0f / timeStep;
	for (auto& [key, manifold] : mManifolds)
	{
		manifold.prepareToSolve();
		manifold.prepareToSolve(invTimeStep);
	}
}

void ContactSolver::solveVelocities(uint32_t velocityIterations) noexcept
{
	for (uint32_t i = 0; i < velocityIterations; ++i)
	{
		for (auto& pair : mManifolds)
		{
			pair.second.solveVelocities();
		}
	}
}

void ContactSolver::solvePositions(uint32_t positionIterations) noexcept
{
	for (uint32_t i = 0; i < positionIterations; ++i)
	{
		for (auto& pair : mManifolds)
		{
			pair.second.solvePositions();
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

void ContactSolver::finishManifoldsUpdate()
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

} // namespace nph
