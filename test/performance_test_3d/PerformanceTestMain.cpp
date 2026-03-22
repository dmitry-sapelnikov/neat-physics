// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025-2026 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

// Includes
#include <test/PerformanceTest.h>

namespace nph
{

template <>
void createTestScene<3>(std::unique_ptr<World<3>>& world)
{
	constexpr float AABB_EXPANSION_FACTOR = 0.1f;
	constexpr float ALLOWED_PENETRATION = 0.01f;
	constexpr uint32_t SOLVER_VELOCITY_ITERATIONS = 10;
	constexpr uint32_t SOLVER_POSITION_ITERATIONS = 3;
	constexpr Vec3 GRAVITY = Vec3(0.0f, -10.0f, 0.0f);
	constexpr float FRICTION = 0.5f;

	const uint32_t LEVELS = 12;
	constexpr uint32_t BODIES_TO_RESERVE = LEVELS * (LEVELS + 1) / 2 + 1; // +1 for the bottom body

	world = std::make_unique<World<3>>(GRAVITY);

	CollisionSettings& collisionSettings = world->getSettings().collisionSettings;
	collisionSettings.setAabbTreeExpansionFactor(AABB_EXPANSION_FACTOR);
	collisionSettings.setSeparationFactor(ALLOWED_PENETRATION);

	SolverSettings& solverSettings = world->getSettings().solverSettings;
	solverSettings.setAllowedPenetration(ALLOWED_PENETRATION);
	solverSettings.setVelocityIterations(SOLVER_VELOCITY_ITERATIONS);
	solverSettings.setPositionIterations(SOLVER_POSITION_ITERATIONS);

	world->reserveBodies(BODIES_TO_RESERVE);

	// Bottom
	world->addBody(
		{ 100.0f, 2.0f, 100.0f },
		0.0f,
		FRICTION,
		{ 0.0f, -1.0f, 0.0f });

	const Vec3 boxSize(2.0f);
	const float boxMass = boxSize.x * boxSize.y * boxSize.z * 1000.0f;
	for (uint32_t i = 0; i < LEVELS; ++i)
	{
		const uint32_t boxesInLevel = LEVELS - i;
		const float yPos = boxSize.y / 2.0f + i * boxSize.y;
		const float startXPos = -((boxesInLevel - 1) * boxSize.x) / 2.0f;
		for (uint32_t j = 0; j < boxesInLevel; ++j)
		{
			const float xPos = startXPos + j * (boxSize.x + 0.2f);
			world->addBody(
				boxSize,
				boxMass,
				FRICTION,
				nph::Vec3(xPos, yPos, 0.0f));
		}
	}
}

} // anonymous namespace

/// 3D Regression test entry point
int wmain()
{
	static constexpr uint32_t SIMULATION_STEPS = 10000;
	return nph::runPerformanceTest<3>(SIMULATION_STEPS, false);
}
