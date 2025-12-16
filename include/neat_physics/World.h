#pragma once

#include <vector>
#include "neat_physics/Body.h"
#include "neat_physics/collision/CollisionSystem.h"

namespace nph
{

/// Physics world
class World
{
public:
	/// Constructor
	/// \param maxBodies Maximum number of bodies in the world; must be > 0
	/// \param gravity Gravity vector applied to all bodies
	explicit World(
		uint32_t maxBodies,
		const Vec2& gravity);

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

	/// Adds a body to the world
	/// \return the added body or nullptr if the body could not be added
	/// (e.g., maximum number of bodies reached)
	Body* addBody(
		const Vec2& size,
		float mass,
		float friction,
		const Vec2& position = {0.0f, 0.0f},
		float rotationRad = 0.0f);

	/// Removes all entities from the world
	void clear() noexcept;

	/// Performs one simulation step
	void doStep(float timeStep);

private:
	/// Applies forces to all bodies
	void applyForces(float timeStep);

	/// Integrates velocities of all bodies
	void integrateVelocities(float timeStep);

	/// Gravity vector
	Vec2 mGravity;

	/// Bodies in the world
	BodyArray mBodies;

	/// Collision system
	CollisionSystem mCollision;
};

} // namespace nph
