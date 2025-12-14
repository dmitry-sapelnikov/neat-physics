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
	/// Constructor
	/// \param maxBodies Maximum number of bodies in the world; asserted to be > 0
	/// \param gravity Gravity vector applied to all bodies
	/// \param velocityIterations Velocity iterations for constraint solvers; asserted to be > 0
	World(
		uint32_t maxBodies,
		const Vec2& gravity,
		uint32_t velocityIterations);

	/// Returns the bodies in the world
	[[nodiscard]] const BodyArray& getBodies() const noexcept
	{
		return mBodies;
	}

	/// Returns the collision system
	[[nodiscard]] const CollisionSystem& getCollision() const noexcept
	{
		return mCollision;
	}

	/// Returns the contact solver
	[[nodiscard]] const ContactSolver& getContactSolver() const noexcept
	{
		return mContactSolver;
	}

	/// Adds a body to the world
	/// \return the added body or nullptr if the body could not be added
	/// (e.g., maximum number of bodies reached)
	Body* addBody(
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

private:
	/// Applies forces to all bodies
	void applyForces(float timeStep);

	/// Integrates velocities of all bodies
	void integrateVelocities(float timeStep);

	/// Gravity vector
	Vec2 mGravity;

	/// Number of velocity iterations for constraint constraints
	uint32_t mVelocityIterations;

	/// Bodies in the world
	BodyArray mBodies;

	/// Collision system
	CollisionSystem mCollision;

	/// Contact solver
	ContactSolver mContactSolver;
};

} // namespace nph
