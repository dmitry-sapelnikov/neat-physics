// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#include <chrono>
#include "Core.h"
#include "Gui.h"
#include "Visualization.h"

namespace nph
{
namespace
{

/// Number of bodies to reserve space for in the physics world
static constexpr uint32_t BODIES_TO_RESERVE = 16;

/// Gravity
static constexpr float GRAVITY = 10.0f;

/// Scene control parameters
struct SceneControl
{
	/// Friction for newly created bodies
	float friction{ 0.0f };

	/// Density of newly created boxes
	float boxDensity{ 200.0f };

	/// Number of boxes along the glass width
	float boxSize{ 8.0f };

	/// Box side ratio (height / width)
	float boxSideRatio{ 0.5f };
};

/// Creates a 'glass-shaped' container
void createGlass(
	nph::World<2>& world,
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
	nph::World<2>& world,
	float glassSize,
	const nph::SceneControl& sceneControl)
{
	nph::Visualization* visualization = nph::Visualization::getInstance();
	assert(visualization != nullptr);

	const bool overImgui =
		ImGui::IsAnyItemHovered() ||
		ImGui::IsWindowHovered(ImGuiHoveredFlags_AnyWindow);

	if (overImgui ||
		!visualization->getMouseInput().leftMouseClicked &&
		!visualization->getMouseInput().rightMouseDown)
	{
		return;
	}

	const float boxSizeX = glassSize / sceneControl.boxSize;
	const float boxSizeY = boxSizeX * sceneControl.boxSideRatio;
	const float boxMass = boxSizeX * boxSizeY * sceneControl.boxDensity;

	Vec3 ray = visualization->getCamera().screenToCameraRay(
		visualization->getMouseInput().position).getNormalized();

	if (std::abs(ray.z) < FLT_EPSILON)
	{
		return;
	}
	const CameraView& cameraView = visualization->getCamera().getView();
	// Ray parameter corresponding to z = 0 plane
	const float t = -cameraView.getPosition().z / ray.z;
	const Vec3 boxLocation = cameraView.getPosition() + t * ray;

	world.addBody(
		{ boxSizeX, boxSizeY },
		boxMass,
		sceneControl.friction,
		{ boxLocation.x, boxLocation.y });
}

/// Draws the help tab in ImGui
void drawHelpTab()
{
	if (ImGui::CollapsingHeader("Help"))
	{
		ImGui::Text("Controls:");
		ImGui::BulletText("Middle mouse button - pan");
		ImGui::BulletText("Mouse wheel - zoom");
		ImGui::BulletText("Left mouse button - add a single box");
		ImGui::BulletText("Right mouse button - add multiple boxes");
		ImGui::Separator();
		ImGui::Text("Notes:");
		ImGui::BulletText(
			"Friction parameter applies only \n"
			"to the newly created objects.");
		ImGui::BulletText(
			"To create walls with nonzero friction,\n"
			"set friction first, then press Reset.");
		ImGui::BulletText("Disable VSync to speed up the simulation.");
	}
}

/// Draws the ImGui scene control tab
void drawSceneControlTab(SceneControl& sceneControl)
{
	if (ImGui::CollapsingHeader(
		"Scene",
		ImGuiTreeNodeFlags_DefaultOpen))
	{
		ImGui::SliderFloat(
			"New Bodies Friction",
			&sceneControl.friction,
			0.0f,
			1.0f,
			"%.1f");

		if (ImGui::CollapsingHeader(
			"New Boxes",
			ImGuiTreeNodeFlags_DefaultOpen))
		{
			ImGui::SliderFloat(
				"Size",
				&sceneControl.boxSize,
				1.0f,
				20.0f,
				"1 / %.0f of glass");

			ImGui::SliderFloat(
				"Side Ratio",
				&sceneControl.boxSideRatio,
				0.1f,
				1.0f,
				"%.2f");

			ImGui::SliderFloat(
				"Density",
				&sceneControl.boxDensity,
				100.0f,
				500.0f,
				"%.0f");
		}
	}
}

/// Draws ImGui controls
void drawGui(
	const nph::World<2>& world,
	float lastPhysicsStepTime,
	nph::WorldDrawSettings& drawSettings,
	SimulationControl& simulationControl,
	SceneControl& sceneControl)
{
	ImGui::SetNextWindowPos(ImVec2(10.0f, 10.0f), ImGuiCond_Once);
	ImGui::SetNextWindowSize(ImVec2(400.0f, 700.0f), ImGuiCond_Once);

	ImGui::Begin("Settings", nullptr, ImGuiWindowFlags_NoCollapse);
	ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.4f);

	drawHelpTab();
	drawVisualizationTab(drawSettings);
	drawStatsTab(world, lastPhysicsStepTime);
	drawSimulationControlTab(simulationControl);
	drawSceneControlTab(sceneControl);

	ImGui::PopItemWidth();
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
		nph::World<2> world({ 0.0f, -nph::GRAVITY }, 1, 1);
		world.reserveBodies(nph::BODIES_TO_RESERVE);

		nph::Visualization* visualization = nph::Visualization::getInstance();
		if (visualization == nullptr)
		{
			return -1;
		}

		visualization->setCameraPosition({ 0.0f, glassSize.y * 0.5f, glassSize.y * 1.5f });
		visualization->setCameraTarget({ 0.0f, glassSize.y * 0.5f, 0.0f });

		ImGuiStyle& style = ImGui::GetStyle();
		style.ItemSpacing.y = 6.0f;

		nph::WorldDrawSettings drawSettings;
		nph::SimulationControl simulationControl;
		nph::SceneControl sceneControl;
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
					sceneControl.friction);
				simulationControl.resetWorld = false;
			}

			nph::addBoxOnMouseClick(
				world,
				glassSize.x,
				sceneControl);

			visualization->startFrame();
			visualization->drawWorld(world, drawSettings);
			nph::drawGui(
				world,
				lastPhyicsStepTime.count(),
				drawSettings,
				simulationControl,
				sceneControl);
			visualization->endFrame();

			visualization->setVSyncEnabled(simulationControl.vSync);

			world.setVelocityIterations(
				uint32_t(simulationControl.velocityIterations));

			world.setPositionIterations(
				uint32_t(simulationControl.positionIterations));

			if (simulationControl.simulationRunning)
			{
				const auto tic = std::chrono::high_resolution_clock::now();
				world.doStep(1.0f / simulationControl.timeStepFrequency);
				const auto toc = std::chrono::high_resolution_clock::now();
				lastPhyicsStepTime = toc - tic;
			}
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
