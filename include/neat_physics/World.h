// A minimalistic 2D physics engine (https://github.com/dmitry-sapelnikov/neat-physics)
// SPDX-FileCopyrightText: 2025 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

#include <vector>
#include "neat_physics/Body.h"
#include "neat_physics/collision/CollisionSystem.h"
#include "neat_physics/dynamics/ContactSolver.h"

namespace nph
{

/// Physics world
class World
{
public:
	/// Body array alias
	using BodyArrayType = BodyArray<2>;

	/// Collision system alias
	using CollisionSystemType = CollisionSystem<2>;

	/// Constructor
	/// \param maxBodies Maximum number of bodies in the world; asserted to be > 0
	/// \param gravity Gravity vector applied to all bodies
	/// \param velocityIterations Velocity iterations for constraint solvers; asserted to be > 0
	World(
		const Vec2& gravity,
		uint32_t velocityIterations,
		uint32_t positionIterations);

	/// Reserves memory for bodies
	/// \note The number of bodies is intentionally limited to uint32_t
	void reserveBodies(uint32_t maxBodies);

	/// Returns the bodies in the world
	[[nodiscard]] const BodyArrayType& getBodies() const noexcept
	{
		return mBodies;
	}

	/// Returns the collision system
	[[nodiscard]] const CollisionSystemType& getCollision() const noexcept
	{
		return mCollision;
	}

	/// Returns the contact solver
	[[nodiscard]] const ContactSolver<2>& getContactSolver() const noexcept
	{
		return mContactSolver;
	}

	/// Adds a body to the world
	/// \return the added body or nullptr if the body could not be added
	/// (e.g., when the number of bodies == uint32_t max value)
	Body<2>* addBody(
		const Vec2& size,
		float mass,
		float friction,
		const Vec2& position = {0.0f, 0.0f},
		float rotationRad = 0.0f);

	/// Clear the world: remove all bodies
	void clear() noexcept;

	/// Perform one simulation step
	void doStep(float dt);

	/// Returns the number of velocity iterations for constraint solvers
	[[nodiscard]] uint32_t getVelocityIterations() const noexcept
	{
		return mVelocityIterations;
	}

	/// Sets the number of velocity solver iterations for constraint solvers
	/// asserts that iterations > 0
	void setVelocityIterations(uint32_t iterations)
	{
		assert(iterations > 0);
		mVelocityIterations = iterations;
	}

	/// Returns the number of position iterations for constraint solvers
	[[nodiscard]] uint32_t getPositionIterations() const noexcept
	{
		return mPositionIterations;
	}

	/// Sets the number of position solver iterations for constraint solvers
	void setPositionIterations(uint32_t iterations)
	{
		mPositionIterations = iterations;
	}

private:
	/// Applies forces to all bodies
	void applyForces(float timeStep);

	/// Integrates positions of all bodies
	void integratePositions(float timeStep);

	/// Gravity vector
	Vec2 mGravity;

	/// Number of velocity iterations for constraint solvers
	uint32_t mVelocityIterations;

	/// Number of position iterations for constraint solvers
	uint32_t mPositionIterations;

	/// Bodies in the world
	BodyArrayType mBodies;

	/// Collision system
	CollisionSystemType mCollision;

	/// Contact solver
	ContactSolver<2> mContactSolver;
};

} // namespace nph
