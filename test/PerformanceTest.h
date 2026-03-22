// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025-2026 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

// Includes
#include <chrono>
#include <framework/Core.h>
#include <framework/Visualization.h>

namespace nph
{

/// Creates the test scene
template <uint16_t D>
void createTestScene(std::unique_ptr<World<D>>& world);

/// Runs the regression test
template <uint16_t D>
int runPerformanceTest(
	uint32_t simulationSteps,
	bool visualize = false) noexcept
{
	assert(simulationSteps > 0);

	constexpr float TIME_STEP = 1.0f / 60.0f;

	try
	{
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
			visualization->setVSyncEnabled(false);
		}

		std::cout << "Total bodies: " << world->getBodyStorage().size() << std::endl;
		auto tic = std::chrono::high_resolution_clock::now();
		for (uint32_t step = 0; step < simulationSteps; ++step)
		{
			world->doStep(TIME_STEP);
			if (step % 60 == 0)
			{
				std::cout << "Step " << step / 60 << "\n";
				uint32_t totalContacts = 0;
				for (const ContactManifold<D>& manifold : world->getContactSolver().getManifolds())
				{
					totalContacts += manifold.getContactCount();
				}
				std::cout << "Total contacts: " << totalContacts << std::endl;
			}

			if (visualization != nullptr)
			{
				visualization->startFrame();
				visualization->drawWorld(*world, {});
				visualization->endFrame();
			}
		}
		auto toc = std::chrono::high_resolution_clock::now();
		std::cout << "\nSimulation time: "
			<< std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic).count()
			<< " ms" << std::endl;

		std::cout << "Press the Enter key to close the window...";
		std::cin.get();
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
