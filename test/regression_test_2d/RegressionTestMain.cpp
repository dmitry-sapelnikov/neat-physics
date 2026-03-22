// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025-2026 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

// Includes
#include <test/RegressionTest.h>

namespace nph
{

/// Creates the test scene
template <>
void createTestScene<2>(std::unique_ptr<World<2>>& world)
{
	constexpr uint32_t SOLVER_VELOCITY_ITERATIONS = 15;
	constexpr uint32_t SOLVER_POSITION_ITERATIONS = 5;
	constexpr uint32_t BODIES_TO_RESERVE = 2048;
	constexpr Vec2 GRAVITY = Vec2(0.0f, -10.0f);

	constexpr float BOTTOM_SIZE = 25.0f;
	constexpr float BOTTOM_THICKNESS = 5.0f;
	constexpr float BOX_BOTTOM_RATIO = 1.0f / 15.0f;
	constexpr int COLUMN_COUNT = 20;
	constexpr int ROW_COUNT = COLUMN_COUNT * 5;
	constexpr float FRICTION = 0.5f;

	world = std::make_unique<World<2>>(GRAVITY);
	SolverSettings& solverSettings = world->getSettings().solverSettings;
	solverSettings.setVelocityIterations(SOLVER_VELOCITY_ITERATIONS);
	solverSettings.setPositionIterations(SOLVER_POSITION_ITERATIONS);

	world->reserveBodies(BODIES_TO_RESERVE);

	// Create a 'glass' with bottom size = BOTTOM_SIZE and 2 sides with
	// height = BOTTOM_SIZE * 2
	
	// Bottom
	world->addBody(
		{ BOTTOM_SIZE + 2.0f * BOTTOM_THICKNESS, BOTTOM_THICKNESS },
		0.0f,
		FRICTION,
		{ 0.0f, -BOTTOM_THICKNESS * 0.5f });

	// Left side
	world->addBody(
		{ BOTTOM_THICKNESS, BOTTOM_SIZE * 2.0f },
		0.0f,
		FRICTION,
		{ -(BOTTOM_SIZE + BOTTOM_THICKNESS) * 0.5f, BOTTOM_SIZE });

	// Right side
	world->addBody(
		{ BOTTOM_THICKNESS, BOTTOM_SIZE * 2.0f},
		0.0f,
		FRICTION,
		{ (BOTTOM_SIZE + BOTTOM_THICKNESS) * 0.5f, BOTTOM_SIZE });

	// Create rng
	std::mt19937 gen(42);
	std::uniform_real_distribution<> distrib(0.5, 1.0);

	const Vec2 boxSize = {
		BOTTOM_SIZE * 0.5f * BOX_BOTTOM_RATIO,
		BOTTOM_SIZE * 0.5f * BOX_BOTTOM_RATIO };

	float startY = boxSize.y * 4.0f;
	float startX = -((COLUMN_COUNT - 1) * boxSize.x) / 2.0f;
	for (int row = 0; row < ROW_COUNT; ++row)
	{
		for (int col = 0; col < COLUMN_COUNT; ++col)
		{
			const Vec2 randomizedSize(
				boxSize.x * static_cast<float>(distrib(gen)),
				boxSize.y * static_cast<float>(distrib(gen)));

			const float randomizedMass =
				randomizedSize.x * randomizedSize.y * 1000.0f;

			const float randomizeFriction =
				std::lerp(0.4f, 0.6f, static_cast<float>(distrib(gen)));

			const float x = startX + col * boxSize.x;
			const float y = startY + row * boxSize.y;
			world->addBody(
				randomizedSize,
				randomizedMass,
				randomizeFriction,
				{ x, y });
		}
	}
}

} // anonymous namespace

/// 2D Regression test entry point
int wmain(int argc, wchar_t** argv)
{
	static constexpr uint32_t SIMULATION_STEPS = 400;
	return nph::runRegressionTest<2>(argc, argv, SIMULATION_STEPS);
}
