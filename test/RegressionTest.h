// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025-2026 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

// Includes
#include <fstream>
#include <filesystem>
#include <framework/Core.h>
#include <framework/Visualization.h>

namespace nph
{

/// Creates the test scene
template <uint16_t D>
void createTestScene(std::unique_ptr<World<D>>& world);

inline void storeBodyState(
	const Body<2>& body,
	std::ofstream& file)
{
	file << "Body " << body.getId() << ": ";
	const Vec2& pos = body.getPosition();
	file << "Pos(" << pos.x << ", " << pos.y << ") ";
	file << "Rot(" << body.getRotation().get() << ")\n";
}

inline void storeBodyState(
	const Body<3>& body,
	std::ofstream& file)
{
	file << "Body " << body.getId() << ": ";

	const Vec3& pos = body.getPosition();
	file << "Pos(" << pos.x << ", " << pos.y << ", " << pos.z << ") ";

	const Quat& rot = body.getRotation().get();
	file << "Rot(" << rot.x << ", " << rot.y << ", " << rot.z << ", " << rot.w << ")\n";
}

/// Runs the regression test
template <uint16_t D>
int runRegressionTest(
	int argc,
	wchar_t** argv,
	uint32_t simulationSteps,
	bool visualize = false) noexcept
{
	assert(simulationSteps > 0);

	constexpr float TIME_STEP = 1.0f / 60.0f;

	try
	{
		if (argc != 2)
		{
			logError("Invalid command line arguments.");
			logError("Correct usage: program.exe path_to_output_directory");
			return -1;
		}

		std::unique_ptr<World<D>> world;
		createTestScene(world);

		Visualization* visualization{ nullptr };
		if (visualize)
		{
			visualization = Visualization::getInstance();
			if (visualization == nullptr)
			{
				logError("Failed to initialize visualization.");
				return -1;
			}
			visualization->setRotationSpeed(D == 2 ? 0.0f : 1.0f);
			visualization->setDepthTest(D != 2);
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

		for (uint32_t step = 0; step < simulationSteps + 1; ++step)
		{
			if (step % simulationSteps == 0)
			{
				resultFile << "Step " << step << ":\n";
				for (uint32_t i = 0; i < world->getBodyStorage().size(); ++i)
				{
					Body<D> body(*world, i);
					storeBodyState(body, resultFile);
				}
				resultFile << "\n";
			}

			world->doStep(TIME_STEP);
			std::cout << "\rProgress: " << (100 * (step + 1) / simulationSteps) << "%";

			if (visualization != nullptr)
			{
				visualization->startFrame();
				visualization->drawWorld(*world, {});
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

} // namespace nph