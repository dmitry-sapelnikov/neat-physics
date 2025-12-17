// A minimalistic 2D physics engine (https://github.com/dmitry-sapelnikov/neat-physics)
// SPDX-FileCopyrightText: 2025 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

// Includes
#include <unordered_map>
#include "neat_physics/collision/CollisionCallback.h"
#include "neat_physics/dynamics/ContactManifold.h"

namespace nph
{

/// Solver for contact constraints between bodies
class ContactSolver : public CollisionCallback
{
public:
	/// Contact manifold container type
	using ContactManifoldsMap = std::unordered_map<uint64_t, ContactManifold>;

	/// Constructor
	ContactSolver(BodyArray& bodies) noexcept;

	/// Clear all contact manifolds
	void clear() noexcept;

	/// Returns the contact manifolds
	[[nodiscard]] const ContactManifoldsMap& getManifolds() const noexcept
	{
		return mManifolds;
	}

	/// Prepares the contact manifolds update
	void prepareManifoldsUpdate() noexcept;

	/// Update a contact manifold
	void onCollision(const CollisionManifold& collisionManifold);

	/// Finishes the contact manifolds update
	void finishManifoldsUpdate();

	/// Prepares the contact solver for velocity solving
	void prepareToSolve(float timeStep) noexcept;

	/// Solves the contact velocities
	void solveVelocities(uint32_t velocityIterations) noexcept;

	/// Solves the contact positions (penetration)
	void solvePositions(uint32_t positionIterations) noexcept;

private:
	/// Reference to the body array
	BodyArray& mBodies;

	/// Persistent contact manifolds
	ContactManifoldsMap mManifolds;
};

}
