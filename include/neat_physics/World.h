// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
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
template <uint16_t D>
class World
{
public:
	/// Constructor
	/// \param maxBodies Maximum number of bodies in the world; asserted to be > 0
	/// \param gravity Gravity vector applied to all bodies
	/// \param velocityIterations Velocity iterations for constraint solvers; asserted to be > 0
	World(
		const Vec<D>& gravity,
		uint32_t velocityIterations,
		uint32_t positionIterations);

	/// Reserves memory for bodies
	/// \note The number of bodies is intentionally limited to uint32_t
	void reserveBodies(uint32_t maxBodies);

	/// Returns the bodies in the world
	[[nodiscard]] const BodyArray<D>& getBodies() const noexcept
	{
		return mBodies;
	}

	/// Returns the collision system
	[[nodiscard]] const CollisionSystem<D>& getCollision() const noexcept
	{
		return mCollision;
	}

	/// Returns the contact solver
	[[nodiscard]] const ContactSolver<D>& getContactSolver() const noexcept
	{
		return mContactSolver;
	}

	/// Adds a body to the world
	/// \return the added body or nullptr if the body could not be added
	/// (e.g., when the number of bodies == uint32_t max value)
	Body<D>* addBody(
		const Vec<D>& size,
		float mass,
		float friction,
		const Vec<D>& position = {},
		const AxisAngle<D>& rotationRad = {});

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
	Vec<D> mGravity;

	/// Number of velocity iterations for constraint solvers
	uint32_t mVelocityIterations;

	/// Number of position iterations for constraint solvers
	uint32_t mPositionIterations;

	/// Bodies in the world
	BodyArray<D> mBodies;

	/// Collision system
	CollisionSystem<D> mCollision;

	/// Contact solver
	ContactSolver<D> mContactSolver;
};

template <uint16_t D>
World<D>::World(
	const Vec<D>& gravity,
	uint32_t velocityIterations,
	uint32_t positionIterations) :

	mGravity(gravity),
	mCollision(mBodies),
	mContactSolver(mBodies)
{
	setVelocityIterations(velocityIterations);
	setPositionIterations(positionIterations);
}

template <uint16_t D>
void World<D>::reserveBodies(uint32_t maxBodies)
{
	const Body<D>* const oldData = mBodies.data();
	mBodies.reserve(maxBodies);
	if (std::ptrdiff_t memoryOffsetInBytes =
		reinterpret_cast<std::byte*>(mBodies.data()) -
		reinterpret_cast<const std::byte*>(oldData);
		memoryOffsetInBytes != 0)
	{
		mContactSolver.onBodiesReallocation(memoryOffsetInBytes);
	}
}

template <uint16_t D>
Body<D>* World<D>::addBody(
	const Vec<D>& size,
	float mass,
	float friction,
	const Vec<D>& position,
	const AxisAngle<D>& rotationRad)
{
	// We limit the number of bodies to uint32_t max value
	if (mBodies.size() == std::numeric_limits<uint32_t>::max())
	{
		return nullptr;
	}

	const Body<D>* const oldData = mBodies.data();
	Body<D>* result = &mBodies.emplace_back(size, mass, friction);
	result->position = position;
	result->rotation.setAngle(rotationRad);

	if (const std::ptrdiff_t memoryOffsetInBytes =
		reinterpret_cast<std::byte*>(mBodies.data()) -
		reinterpret_cast<const std::byte*>(oldData);
		memoryOffsetInBytes != 0)
	{
		mContactSolver.onBodiesReallocation(memoryOffsetInBytes);
	}
	return result;
}

template <uint16_t D>
void World<D>::clear() noexcept
{
	mBodies.clear();
	mContactSolver.clear();
}

template <uint16_t D>
void World<D>::doStep(float timeStep)
{
	assert(timeStep > 0.0f);
	applyForces(timeStep);

	mContactSolver.prepareManifoldsUpdate();
	mCollision.update(mContactSolver);
	mContactSolver.finishManifoldsUpdate();

	mContactSolver.prepareToSolve();
	mContactSolver.solveVelocities(mVelocityIterations);
	integratePositions(timeStep);
	// Solving of positions is intetionally done after the integration step
	mContactSolver.solvePositions(mPositionIterations);
}

template <uint16_t D>
void World<D>::applyForces(float timeStep)
{
	for (auto& body : mBodies)
	{
		body.linearVelocity += (!body.isStatic()) * timeStep * mGravity;
	}
}

template <uint16_t D>
void World<D>::integratePositions(float timeStep)
{
	for (auto& body : mBodies)
	{
		body.position += timeStep * body.linearVelocity;
		/// \todo generalize for 3D
		body.rotation.setAngle(
			body.rotation.getAngle() + timeStep * body.angularVelocity);
	}
}

} // namespace nph
