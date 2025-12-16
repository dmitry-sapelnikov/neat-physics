#pragma once

// Includes
#include <unordered_map>
#include "neat_physics/dynamics/ContactManifold.h"
#include "neat_physics/Body.h"

namespace nph
{

/// Solver for contact constraints between bodies
class ContactSolver
{
public:
	/// Collision manifold container type
	using CollisionManifoldsType = std::vector<CollisionManifold>;

	/// Contact manifold container type
	using ContactManifoldsType = std::unordered_map<uint64_t, ContactManifold>;

	/// Constructor
	ContactSolver(
		BodyArray& bodies,
		const CollisionManifoldsType& collisionManifolds);

	/// Clear all contact manifolds
	void clear() noexcept;

	/// Returns the contact manifolds
	[[nodiscard]] const ContactManifoldsType& getManifolds() const noexcept
	{
		return mManifolds;
	}

	/// Prepares the contact solver for velocity solving
	void prepareToSolve(float timeStep);

	/// Solves the contact velocities
	void solveVelocities(uint32_t velocityIterations);

	/// Solves the contact positions (penetration)
	void solvePositions(uint32_t positionIterations);

private:
	/// Updates the persistent contact manifolds
	void updateManifolds();

	/// Reference to the body array
	BodyArray& mBodies;

	/// Reference to the contact manifolds
	const CollisionManifoldsType& mCollisionManifolds;

	/// Persistent contact manifolds
	ContactManifoldsType mManifolds;
};

}
