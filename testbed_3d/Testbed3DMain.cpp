// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025-2026 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#include <chrono>
#include "Testbed.h"

namespace nph
{
namespace
{

/// Number of bodies to reserve space for in the physics world
static constexpr uint32_t BODIES_TO_RESERVE = 256;

/// Gravity
static constexpr float GRAVITY = 10.0f;

/// Draws the help tab in ImGui
void drawHelpTab()
{
	if (ImGui::CollapsingHeader("Help"))
	{
		ImGui::Text("Controls:");
		ImGui::BulletText("Left mouse button - rotate");
		ImGui::BulletText("Middle mouse button - pan");
		ImGui::BulletText("Mouse wheel - zoom");
		ImGui::BulletText("Right mouse button - add multiple boxes");
		ImGui::Separator();
		ImGui::Text("Notes:");
		ImGui::BulletText("Disable VSync to speed up the simulation.");
	}
}

void createPyramid(
	const uint32_t levels,
	Vec3 boxSize,
	nph::World<3>& world)
{
	const float boxMass = boxSize.x * boxSize.y * boxSize.z * 1000.0f;
	const float boxFriction = 0.5f;
	for (uint32_t i = 0; i < levels; ++i)
	{
		const uint32_t boxesInLevel = levels - i;
		const float yPos = boxSize.y / 2.0f + i * boxSize.y;
		const float startXPos = -((boxesInLevel - 1) * boxSize.x) / 2.0f;
		for (uint32_t j = 0; j < boxesInLevel; ++j)
		{
			const float xPos = startXPos + j * boxSize.x;
			world.addBody(
				boxSize,
				boxMass,
				boxFriction,
				nph::Vec3(xPos, yPos, 0.0f));
		}
	}
}

void addBoxOnMouseClick(
	nph::World<3>& world)
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

	Vec3 ray = visualization->getCamera().screenToCameraRay(
		visualization->getMouseInput().position).getNormalized();

	if (std::abs(ray.z) < FLT_EPSILON)
	{
		return;
	}
	const CameraView& cameraView = visualization->getCamera().getView();
	const nph::Vec3 boxSize(1.0f, 0.8f, 0.5f);
	const float boxMass = boxSize.x * boxSize.y * boxSize.z * 2000.0f;
	nph::Body<3> box = world.addBody(
		boxSize,
		boxMass,
		0.5f,
		cameraView.getPosition() + 2.0 * ray);

	box.setLinearVelocity(20.0 * ray);
}

} // anonymous namespace
} // namespace nph

/// The application entry point
int main()
{
	auto worldCreator =
		[]()
		{
			auto result = std::make_unique<nph::World<3>>(
				nph::Vec3(0.0f, -nph::GRAVITY, 0.0f));
			result->reserveBodies(nph::BODIES_TO_RESERVE);
			return result;
		};

	nph::Testbed<3> testbed(worldCreator);

	auto sceneCreationCallback =
		[](nph::World<3>& world)
		{
			world.addBody(
				nph::Vec3(50.0f, 1.0f, 50.0f),
				0.0f,
				0.5f,
				nph::Vec3(0.0f, -0.5f, 0.0f));

			nph::createPyramid(20, nph::Vec3(1.0f, 1.0f, 1.0f), world);
		};
	testbed.setSceneCreationCallback(sceneCreationCallback);

	auto visualizationSetupCallback =
		[](nph::Visualization& visualization)
		{
			visualization.setRotationSpeed(0.3f);
			visualization.setCameraPosition({ 30.0f, 10.0f, 30.0f });
			visualization.setCameraTarget({ 0.0f, 5.0f, 0.0f });

			ImGuiStyle& style = ImGui::GetStyle();
			style.ItemSpacing.y = 6.0f;
		};
	testbed.setVisualizationSetupCallback(visualizationSetupCallback);

	auto mouseClickCallback =
		[](nph::World<3>& world)
		{
			nph::addBoxOnMouseClick(world);
		};
	testbed.setRightMouseDownCallback(mouseClickCallback);


	auto guiCallback =
		[]()
		{
			nph::drawHelpTab();
		};
	testbed.setGuiDrawCallback(guiCallback);

	nph::SimulationControl simulationControl;
	simulationControl.velocityIterations = 10;
	simulationControl.positionIterations = 3;
	nph::WorldDrawSettings drawSettings;
	return testbed.run(simulationControl, drawSettings);
}
