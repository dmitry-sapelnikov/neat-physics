// Includes
#include "neat_physics/dynamics/ContactSolver.h"

namespace nph
{

ContactSolver::ContactSolver(
	BodyArray& bodies,
	const CollisionManifoldsType& collisionManifolds) :

	mBodies(bodies),
	mCollisionManifolds(collisionManifolds)
{
}

void ContactSolver::clear() noexcept
{
	mManifolds.clear();
}

void ContactSolver::solve(
	float timeStep,
	uint32_t velocityIterations)
{
	updateManifolds();

	const float invTimeStep = 1.0f / timeStep;
	for (auto& [key, manifold] : mManifolds)
	{
		manifold.prepareToSolve(invTimeStep);
	}

	for (uint32_t i = 0; i < velocityIterations; ++i)
	{
		for (auto& [key, manifold] : mManifolds)
		{
			manifold.solveVelocities();
		}
	}
}

void ContactSolver::updateManifolds()
{
	for (auto& pair : mManifolds)
	{
		pair.second.markObsolete();
	}

	Body* start = mBodies.data();
	for (const auto& manifold : mCollisionManifolds)
	{
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

	// Remove obsolete manifolds
	std::erase_if(
		mManifolds,
		[](const auto& pair)
		{
			return pair.second.isObsolete();
		});
}

} // namespace nph
