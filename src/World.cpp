#include "neat_physics/World.h"

namespace nph
{

World::World(
	uint32_t maxBodies,
	const Vec2& gravity) :

	mGravity(gravity)
{
	assert(maxBodies > 0);
	mBodies.reserve(maxBodies);
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
}

void World::doStep(float timeStep)
{
	assert(timeStep > 0.0f);
	applyForces(timeStep);
	integrateVelocities(timeStep);
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
