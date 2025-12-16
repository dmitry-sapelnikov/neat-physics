// A minimalistic 2D physics engine (https://github.com/dmitry-sapelnikov/neat-physics)
// SPDX-FileCopyrightText: 2025 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#include "neat_physics/World.h"

namespace nph
{

World::World(
	uint32_t maxBodies,
	const Vec2& gravity,
	uint32_t velocityIterations,
	uint32_t positionIterations) :

	mGravity(gravity),
	mCollision(mBodies),
	mContactSolver(mBodies, mCollision.getCollisionManifolds())
{
	assert(maxBodies > 0);
	mBodies.reserve(maxBodies);
	setVelocityIterations(velocityIterations);
	setPositionIterations(positionIterations);
}

Body* World::addBody(
	const Vec2& size,
	float mass,
	float friction,
	const Vec2& position,
	float rotationRad)
{
	if (mBodies.size() == mBodies.capacity())
	{
		return nullptr;
	}

	Body* result = &mBodies.emplace_back(size, mass, friction);
	result->position = position;
	result->rotation.setAngle(rotationRad);
	return result;
}

void World::clear() noexcept
{
	mBodies.clear();
	mContactSolver.clear();
}

void World::doStep(float timeStep)
{
	assert(timeStep > 0.0f);
	applyForces(timeStep);
	mCollision.update();
	mContactSolver.prepareToSolve(timeStep);
	mContactSolver.solveVelocities(mVelocityIterations);
	integrateVelocities(timeStep);
	mContactSolver.solvePositions(mPositionIterations);
}

void World::applyForces(float timeStep)
{
	for (auto& body : mBodies)
	{
		body.linearVelocity += (!body.isStatic()) * timeStep * mGravity;
	}
}

void World::integrateVelocities(float timeStep)
{
	for (auto& body : mBodies)
	{
		body.position += timeStep * body.linearVelocity;
		body.rotation.setAngle(
			body.rotation.getAngle() + timeStep * body.angularVelocity);
	}
}

} // namespace nph
