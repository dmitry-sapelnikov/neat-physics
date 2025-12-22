// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

// Includes
#include "Gui.h"

namespace nph
{

/// Draws the visualization tab in ImGui
void drawVisualizationTab(nph::WorldDrawSettings& drawSettings)
{
	if (ImGui::CollapsingHeader("Visualization"))
	{
		ImGui::Checkbox(
			"AABBs",
			&drawSettings.aabbs);

		ImGui::Checkbox(
			"Body Frames",
			&drawSettings.bodyFrames);

		ImGui::SliderFloat(
			"Body Frame Size",
			&drawSettings.bodyFrameSize,
			0.1f,
			1.0f);

		ImGui::Checkbox(
			"Contacts",
			&drawSettings.contacts);

		ImGui::SliderFloat(
			"Contact Size",
			&drawSettings.contactSize,
			2.0f,
			10.0f);

		ImGui::Checkbox(
			"Body Velocities",
			&drawSettings.bodyVelocities);

		ImGui::SliderFloat(
			"Velocity Arrow Size",
			&drawSettings.bodyVelocityArrowSize,
			0.1f,
			0.5f);
	}
}

/// Draws the ImGui simulation control tab
void drawSimulationControlTab(
	SimulationControl& simulationControl)
{
	if (ImGui::CollapsingHeader(
		"Simulation",
		ImGuiTreeNodeFlags_DefaultOpen))
	{
		const ImVec2 buttonsSize{
			ImGui::GetWindowWidth() * 0.4f,
			0.0f };
		simulationControl.resetWorld = ImGui::Button("Reset", buttonsSize);

		if (ImGui::Button(
			simulationControl.simulationRunning ? "Pause" : "Resume",
			buttonsSize))
		{
			simulationControl.simulationRunning =
				!simulationControl.simulationRunning;
		}

		ImGui::SliderFloat(
			"Time Step Frequency",
			&simulationControl.timeStepFrequency,
			30.0f,
			100.0f,
			"%.0f Hz");

		ImGui::SliderInt(
			"Velocity Iterations",
			&simulationControl.velocityIterations,
			1,
			50);

		ImGui::SliderInt(
			"Position Iterations",
			&simulationControl.positionIterations,
			0,
			50);

		ImGui::Checkbox(
			"VSync",
			&simulationControl.vSync);
	}
}

} // namespace nph
