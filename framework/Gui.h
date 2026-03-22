// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025-2026 Dmitry Sapelnikov
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

	/// FPS-independent physics flag
	bool fpsIndependentPhysics{ false };

	/// Reset world flag
	bool resetWorld{ true };

	/// Simulation running flag
	bool simulationRunning{ true };

	/// Do a single simulation step
	bool doSimulationStep{ false };

	/// Time step frequency
	float timeStepFrequency{ 60.0f };

	/// Velocity solver iterations
	int velocityIterations{ 15 };

	/// Position solver iterations
	int positionIterations{ 3 };
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
			world.getBodyStorage().size());

		ImGui::Text(
			"Contact pairs: %zu",
			world.getContactSolver().getManifolds().size());

		size_t totalContacts = 0;
		size_t persistentContacts = 0;
		float maxPenetration = 0.0f;
		for (const auto& manifold : world.getContactSolver().getManifolds())
		{
			totalContacts += manifold.getContactCount();
			for (uint32_t i = 0; i < manifold.getContactCount(); ++i)
			{
				const auto& contact = manifold.getContact(i);
				persistentContacts += contact.isPersistent();
				maxPenetration = std::max(
					contact.getPoint().penetration,
					maxPenetration);
			}
		}

		ImGui::Text(
			"Physics Time: %.3f ms",
			lastPhysicsStepTime * 1000.0f);

		ImGui::Text(
			"Physics FPS: %.1f",
			1.0f / lastPhysicsStepTime);

		ImGui::Text("Persistent/total contacts: %zu / %zu",
			persistentContacts,
			totalContacts);

		ImGui::Text("Max Penetration : %.3f", maxPenetration);
	}
}

} // namespace nph
