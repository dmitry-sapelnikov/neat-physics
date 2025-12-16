// A minimalistic 2D physics engine (https://github.com/dmitry-sapelnikov/neat-physics)
// SPDX-FileCopyrightText: 2025 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#include <chrono>
#include "Core.h"
#include "Visualization.h"
#include "neat_physics/World.h"

namespace nph
{
namespace
{

/// Maximum number of bodies in the physics world
static constexpr uint32_t MAX_BODIES = 5000;

/// Gravity
static constexpr float GRAVITY = 10.0f;

/// Simulation step
static constexpr float SIMULATION_STEP = 1.0f / 60.0f;

/// Simulation control parameters
struct SimulationControl
{
	bool vSync{ true };
	bool resetWorld{ true };
	float friction{ 0.0f };
	float boxDensity{ 200.0f };
	float boxSize{ 8.0f };
	float boxSideRatio{ 0.5f };
	float timeStepFrequency{ 50.0f };
	int velocityIterations{ 20 };
	int positionIterations{ 10 };
};

/// Creates a 'glass-shaped' container
void createGlass(
	nph::World& world,
	const nph::Vec2& glassSize,
	float glassThickness,
	float friction)
{
	// Create a static 'glass' made of 3 bodies: bottom and 2 sides
	const float bottomSize = glassSize.x * 20.0f;
	const float bottomThickness = glassThickness * 10.0f;
	world.addBody(
		{ bottomSize, bottomThickness },
		0.0f,
		friction,
		{ 0.0f, -bottomThickness * 0.5f });

	world.addBody(
		{ glassThickness, glassSize.y },
		0.0f,
		friction,
		{ -(glassSize.x + glassThickness) * 0.5f, 0.5f * glassSize.y });

	world.addBody(
		{ glassThickness, glassSize.y },
		0.0f,
		friction,
		{ (glassSize.x + glassThickness) * 0.5f, 0.5f * glassSize.y });
}

void addBoxOnMouseClick(
	nph::World& world,
	float glassSize,
	const nph::SimulationControl& simulationControl)
{
	nph::Visualization* visualization = nph::Visualization::getInstance();
	assert(visualization != nullptr);

	const bool overImgui =
		ImGui::IsAnyItemHovered() ||
		ImGui::IsWindowHovered(ImGuiHoveredFlags_AnyWindow);

	if (overImgui ||
		!visualization->getInput().leftMouseClicked &&
		!visualization->getInput().rightMouseDown)
	{
		return;
	}

	const float boxSizeX = glassSize / simulationControl.boxSize;
	const float boxSizeY = boxSizeX * simulationControl.boxSideRatio;
	const float boxMass = boxSizeX * boxSizeY * simulationControl.boxDensity;

	world.addBody(
		{ boxSizeX, boxSizeY },
		boxMass,
		simulationControl.friction,
		visualization->getCursorPositionWorld());
}

/// Draws ImGui controls
void drawGui(
	const nph::World& world,
	float lastPhysicsStepTime,
	nph::WorldDrawSettings& drawSettings,
	float bottomSize,
	SimulationControl& simulationControl)
{
	ImGui::SetNextWindowPos(ImVec2(10.0f, 10.0f), ImGuiCond_Once);
	ImGui::SetNextWindowSize(ImVec2(400.0f, 700.0f), ImGuiCond_Once);

	ImGui::Begin("Settings", nullptr, ImGuiWindowFlags_NoCollapse);

	ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.33f);

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

		float maxAllowedPenetration = 0.1f * (bottomSize / 8.0f);

		// Set text color based on penetration
		ImGui::PushStyleColor(
			ImGuiCol_Text,
			maxPenetration > maxAllowedPenetration ?
				ImVec4(1.0f, 0.0f, 0.0f, 1.0f) :
				ImVec4(0.0f, 1.0f, 0.0f, 1.0f));

		ImGui::Text(
			"Max Penetration: %.4f / %.4f",
			maxPenetration,
			maxAllowedPenetration);

		ImGui::PopStyleColor();
	}

	if (ImGui::CollapsingHeader(
		"Simulation Control",
		ImGuiTreeNodeFlags_DefaultOpen))
	{
		ImGui::Indent(20.0f);

		ImGui::Checkbox(
			"VSync",
			&simulationControl.vSync);

		// Add 'World' splitter
		if (ImGui::CollapsingHeader("World", ImGuiTreeNodeFlags_DefaultOpen))
		{
			simulationControl.resetWorld = ImGui::Button("Reset");

			ImGui::SliderFloat(
				"New Bodies Friction",
				&simulationControl.friction,
				0.0f,
				1.0f,
				"%.1f");

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
		}

		if (ImGui::CollapsingHeader(
			"New Boxes",
			ImGuiTreeNodeFlags_DefaultOpen))
		{
			ImGui::SliderFloat(
				"Size",
				&simulationControl.boxSize,
				1.0f,
				20.0f,
				"1 / %.0f of glass");

			ImGui::SliderFloat(
				"Side Ratio",
				&simulationControl.boxSideRatio,
				0.1f,
				1.0f,
				"%.2f");

			ImGui::SliderFloat(
				"Density",
				&simulationControl.boxDensity,
				100.0f,
				500.0f,
				"%.0f");
		}
		ImGui::Unindent(20.0f);
	}

	ImGui::End();
}

} // anonymous namespace
} // namespace nph

/// The application entry point
int main()
{
	try
	{
		const nph::Vec2 glassSize{ nph::GRAVITY * 0.5f, nph::GRAVITY };
		nph::World world(nph::MAX_BODIES, { 0.0f, -nph::GRAVITY }, 1, 1);

		nph::Visualization* visualization = nph::Visualization::getInstance();
		if (visualization == nullptr)
		{
			return -1;
		}
		visualization->setCameraZoom(int(glassSize.x * 2.0f));
		visualization->setCameraPan(nph::Vec2(0.0f, glassSize.y * 0.5f));
		ImGuiStyle& style = ImGui::GetStyle();
		style.ItemSpacing.y = 6.0f;

		nph::WorldDrawSettings drawSettings;
		nph::SimulationControl simulationControl;
		std::chrono::duration<float> lastPhyicsStepTime{ 0.0 };
		while (visualization->isRunning())
		{
			if (simulationControl.resetWorld)
			{
				world.clear();
				nph::createGlass(
					world,
					glassSize,
					nph::GRAVITY * 0.05f,
					simulationControl.friction);
				simulationControl.resetWorld = false;
			}

			nph::addBoxOnMouseClick(
				world,
				glassSize.x,
				simulationControl);

			visualization->startFrame();
			visualization->drawWorld(world, drawSettings);
			nph::drawGui(
				world,
				lastPhyicsStepTime.count(),
				drawSettings,
				glassSize.x,
				simulationControl);
			visualization->endFrame();

			visualization->setVSyncEnabled(simulationControl.vSync);

			world.setVelocityIterations(
				uint32_t(simulationControl.velocityIterations));

			world.setPositionIterations(
				uint32_t(simulationControl.positionIterations));

			const auto tic = std::chrono::high_resolution_clock::now();
			world.doStep(1.0f / simulationControl.timeStepFrequency);
			const auto toc = std::chrono::high_resolution_clock::now();
			lastPhyicsStepTime = toc - tic;
		}
		return 0;
	}
	catch (const std::exception& exception)
	{
		nph::logError("Exception caught: ", exception.what());
	}
	catch (...)
	{
		nph::logError("Unknown exception caught.");
	}
	return -1;
}
