// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

// Includes
#include "Visualization.h"

namespace nph
{

/// Simulation control parameters
struct SimulationControl
{
	/// V-Sync flag
	bool vSync{ true };

	/// Reset world flag
	bool resetWorld{ true };

	/// Simulation running flag
	bool simulationRunning{ true };

	/// Time step frequency
	float timeStepFrequency{ 50.0f };

	/// Velocity solver iterations
	int velocityIterations{ 30 };

	/// Position solver iterations
	int positionIterations{ 10 };
};

void drawVisualizationTab(WorldDrawSettings& drawSettings);

void drawSimulationControlTab(SimulationControl& simulationControl);

template <uint16_t D>
void drawStatsTab(
	const nph::World<D>& world,
	float lastPhysicsStepTime)
{
	if (ImGui::CollapsingHeader("Stats"))
	{
		ImGui::Text(
			"Bodies: %zu",
			world.getBodies().size());

		ImGui::Text(
			"Contacts: %zu",
			world.getContactSolver().getManifolds().size());

		ImGui::Text(
			"Physics Time: %.3f ms",
			lastPhysicsStepTime * 1000.0f);

		ImGui::Text(
			"Physics FPS: %.1f",
			1.0f / lastPhysicsStepTime);

		float maxPenetration = 0.0f;
		for (const auto& manifold : world.getContactSolver().getManifolds())
		{
			for (uint32_t i = 0; i < manifold.second.getContactCount(); ++i)
			{
				const auto& contact = manifold.second.getContact(i);
				if (contact.getPoint().penetration > maxPenetration)
				{
					maxPenetration = contact.getPoint().penetration;
				}
			}
		}

		ImGui::Text("Max Penetration : %.3f", maxPenetration);
	}
}

} // namespace nph
