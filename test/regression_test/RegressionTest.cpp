// A minimalistic 2D physics engine (https://github.com/dmitry-sapelnikov/neat-physics)
// SPDX-FileCopyrightText: 2025 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

// Includes
#include <filesystem>
#include <fstream>
#include <random>
#include "neat_physics/World.h"
#include "Core.h"
#include "Visualization.h"

using namespace nph;

namespace
{

/// Creates the test scene
void createTestScene(World& world)
{
	constexpr float BOTTOM_SIZE = 25.0f;
	constexpr float BOTTOM_THICKNESS = 5.0f;
	constexpr float BOX_BOTTOM_RATIO = 1.0f / 15.0f;
	constexpr int COLUMN_COUNT = 20;
	constexpr int ROW_COUNT = COLUMN_COUNT * 5;
	constexpr float FRICTION = 0.5f;

	// Create a 'glass' with bottom size = BOTTOM_SIZE and 2 sides with
	// height = BOTTOM_SIZE * 2
	
	// Bottom
	world.addBody(
		{ BOTTOM_SIZE + 2.0f * BOTTOM_THICKNESS, BOTTOM_THICKNESS },
		0.0f,
		FRICTION,
		{ 0.0f, -BOTTOM_THICKNESS * 0.5f });

	// Left side
	world.addBody(
		{ BOTTOM_THICKNESS, BOTTOM_SIZE * 2.0f },
		0.0f,
		FRICTION,
		{ -(BOTTOM_SIZE + BOTTOM_THICKNESS) * 0.5f, BOTTOM_SIZE });

	// Right side
	world.addBody(
		{ BOTTOM_THICKNESS, BOTTOM_SIZE * 2.0f},
		0.0f,
		FRICTION,
		{ (BOTTOM_SIZE + BOTTOM_THICKNESS) * 0.5f, BOTTOM_SIZE });

	// Create rng
	std::mt19937 gen(42);
	std::uniform_real_distribution<> distrib(0.5, 1.0);

	const Vec2 boxDensity = {
		BOTTOM_SIZE * 0.5f * BOX_BOTTOM_RATIO,
		BOTTOM_SIZE * 0.5f * BOX_BOTTOM_RATIO };

	float startY = boxDensity.y * 4.0f;
	float startX = -((COLUMN_COUNT - 1) * boxDensity.x) / 2.0f;
	for (int row = 0; row < ROW_COUNT; ++row)
	{
		for (int col = 0; col < COLUMN_COUNT; ++col)
		{
			const Vec2 randomizedSize(
				boxDensity.x * static_cast<float>(distrib(gen)),
				boxDensity.y * static_cast<float>(distrib(gen)));

			const float randomizedMass =
				randomizedSize.x * randomizedSize.y * 1000.0f;

			const float randomizeFriction =
				std::lerp(0.4f, 0.6f, static_cast<float>(distrib(gen)));

			const float x = startX + col * boxDensity.x;
			const float y = startY + row * boxDensity.y;
			world.addBody(
				randomizedSize,
				randomizedMass,
				randomizeFriction,
				{ x, y });
		}
	}
}

} // anonymous namespace

/// A little regression test that runs a simulation and dumps body positions
int wmain(int argc, wchar_t** argv)
{
	constexpr float TIME_STEP = 1.0f / 60.0f;
	constexpr uint32_t MAX_BODIES = 4096;
	constexpr bool USE_VISUALIZATION = false;

	try
	{
		if (argc != 2)
		{
			logError("Invalid command line arguments.");
			logError("Correct usage: program.exe path_to_output_directory");
			return -1;
		}

		World world(MAX_BODIES, Vec2(0.0f, -10.0f), 15, 5);
		createTestScene(world);

		Visualization* visualization{ nullptr };
		if (USE_VISUALIZATION)
		{
			visualization = Visualization::getInstance();
			if (visualization == nullptr)
			{
				logError("Failed to initialize visualization.");
				return -1;
			}
			visualization->setCameraZoom(80);
			visualization->setCameraPan({ 0.0f, 20.0f });
		}

		std::filesystem::path outputDirectory =
			std::filesystem::canonical(std::wstring(argv[1]));

		auto resultFile =
			std::ofstream(outputDirectory.wstring() + L"\\results.txt");
		if (!resultFile)
		{
			logError("Failed to open results file.");
			return -1;
		}

		int maxSteps = 400;
		int dumpInterval = 10;
		for (int step = 0; step < maxSteps; ++step)
		{
			if (step % dumpInterval == 0)
			{
				resultFile << "Step " << step << ":\n";
				for (size_t i = 0; i < world.getBodies().size(); ++i)
				{
					const Body& body = world.getBodies()[i];
					resultFile << "Body " << i << ": ";
					resultFile << "Pos(" << body.position.x << ", " << body.position.y << ") ";
					resultFile << "Rot(" << body.rotation.getAngle() << ")\n";
				}
				resultFile << "\n";
			}

			world.doStep(TIME_STEP);
			std::cout << "\rProgress: " << (100 * (step + 1) / maxSteps) << "%";

			if (visualization != nullptr)
			{
				visualization->startFrame();
				visualization->drawWorld(world, {});
				visualization->endFrame();
			}
		}
		return 0;
	}
	catch (const std::exception& exception)
	{
		logError("Exception caught: ", exception.what());
	}
	catch (...)
	{
		logError("Unknown exception caught.");
	}
	return -1;
}
