// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025-2026 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

// Includes
#include <test/RegressionTest.h>

namespace nph
{

template <>
void createTestScene<3>(std::unique_ptr<World<3>>& world)
{
	constexpr uint32_t SOLVER_VELOCITY_ITERATIONS = 10;
	constexpr uint32_t SOLVER_POSITION_ITERATIONS = 3;
	constexpr Vec3 GRAVITY = Vec3(0.0f, -10.0f, 0.0f);

	constexpr float BOTTOM_SIZE = 20.0f;
	constexpr float BOTTOM_THICKNESS = 1.0f;
	constexpr float BOX_BOTTOM_RATIO = 1.0f / 8.0f;
	constexpr int BOXES_XZ_COUNT = 10;
	constexpr int BOXES_Y_COUNT = BOXES_XZ_COUNT * 5;
	constexpr float FRICTION = 0.5f;
	constexpr float HEIGHT = BOTTOM_SIZE * 1.2;

	constexpr uint32_t BODIES_TO_RESERVE =
		BOXES_XZ_COUNT * BOXES_XZ_COUNT * BOXES_Y_COUNT + 5;

	world = std::make_unique<World<3>>(GRAVITY);
	SolverSettings& solverSettings = world->getSettings().solverSettings;
	solverSettings.setVelocityIterations(SOLVER_VELOCITY_ITERATIONS);
	solverSettings.setPositionIterations(SOLVER_POSITION_ITERATIONS);

	world->reserveBodies(BODIES_TO_RESERVE);

	// Create a 'glass' with bottom size = BOTTOM_SIZE and 2 sides with
	
	// Bottom
	world->addBody(
		{ BOTTOM_SIZE + 2.0f * BOTTOM_THICKNESS, BOTTOM_THICKNESS, BOTTOM_SIZE + 2.0f * BOTTOM_THICKNESS },
		0.0f,
		FRICTION,
		{ 0.0f, -BOTTOM_THICKNESS * 0.5f, 0.0f });

	// Left side
	world->addBody(
		{ BOTTOM_THICKNESS, HEIGHT, BOTTOM_SIZE },
		0.0f,
		FRICTION,
		{ -(BOTTOM_SIZE + BOTTOM_THICKNESS) * 0.5f, HEIGHT * 0.5f, 0.0f });

	// Right side
	world->addBody(
		{ BOTTOM_THICKNESS, HEIGHT, BOTTOM_SIZE },
		0.0f,
		FRICTION,
		{ (BOTTOM_SIZE + BOTTOM_THICKNESS) * 0.5f, HEIGHT * 0.5f, 0.0f });

	// Front side
	world->addBody(
		{ BOTTOM_SIZE, HEIGHT, BOTTOM_THICKNESS },
		0.0f,
		FRICTION,
		{ 0.0f, HEIGHT * 0.5f, (BOTTOM_SIZE + BOTTOM_THICKNESS) * 0.5f });

	// Back side
	world->addBody(
		{ BOTTOM_SIZE, HEIGHT, BOTTOM_THICKNESS },
		0.0f,
		FRICTION,
		{ 0.0f, HEIGHT * 0.5f, -(BOTTOM_SIZE + BOTTOM_THICKNESS) * 0.5f });

	// Create rng
	std::mt19937 gen(0);
	std::uniform_real_distribution<> distr(0.0, 1.0);

	const Vec2 boxSize = {
		BOTTOM_SIZE * 0.5f * BOX_BOTTOM_RATIO,
		BOTTOM_SIZE * 0.5f * BOX_BOTTOM_RATIO };

	float startY = boxSize.y * 4.0f;
	float startX = -((BOXES_XZ_COUNT - 1) * boxSize.x) / 2.0f;
	for (int yi = 0; yi < BOXES_Y_COUNT; ++yi)
	{
		for (int xi = 0; xi < BOXES_XZ_COUNT; ++xi)
		{
			for (int zi = 0; zi < BOXES_XZ_COUNT; ++zi)
			{
				const Vec3 size(
					boxSize.x * std::lerp(0.5f, 1.0f, static_cast<float>(distr(gen))),
					boxSize.y * std::lerp(0.5f, 1.0f, static_cast<float>(distr(gen))),
					boxSize.x * std::lerp(0.5f, 1.0f, static_cast<float>(distr(gen))));

				const float mass = size.x * size.y * size.z * 1000.0f;

				const float friction =
					std::lerp(0.4f, 0.6f, static_cast<float>(distr(gen)));

				const Vec3 position(
					startX + xi * boxSize.x,
					startY + yi * boxSize.y,
					-((BOXES_XZ_COUNT - 1) * boxSize.x) / 2.0f + zi * boxSize.x);

				const Vec3 axisAngle(
					std::lerp(-1.0f, 1.0f, static_cast<float>(distr(gen))),
					std::lerp(-1.0f, 1.0f, static_cast<float>(distr(gen))),
					std::lerp(-1.0f, 1.0f, static_cast<float>(distr(gen))));

				world->addBody(
					size,
					mass,
					friction,
					position,
					axisAngle);
			}
		}
	}
}

} // anonymous namespace

/// 3D Regression test entry point
int wmain(int argc, wchar_t** argv)
{
	static constexpr uint32_t SIMULATION_STEPS = 400;
	return nph::runRegressionTest<3>(argc, argv, SIMULATION_STEPS);
}
