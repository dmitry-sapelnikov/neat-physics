#include "neat_physics/World.h"

namespace nph
{

World::World(uint32_t maxBodies)
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

} // namespace nph
