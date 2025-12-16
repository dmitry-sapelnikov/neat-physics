#include "Core.h"
#include "Visualization.h"
#include "neat_physics/World.h"

using namespace nph;
namespace
{

/// Maximum number of bodies in the physics world
static constexpr uint32_t MAX_BODIES = 1000;

/// Gravity
static constexpr float GRAVITY = 10.0f;

/// Simulation step
static constexpr float SIMULATION_STEP = 1.0f / 60.0f;

/// Creates a 'glass-shaped' container
void createGlass(
	nph::World& world,
	const nph::Vec2& glassSize,
	float glassThickness,
	float friction)
{
	// Create a static 'glass' made of 3 bodies: bottom and 2 sides
	world.addBody(
		{ glassSize.x + 2.0f * glassThickness, glassThickness },
		0.0f,
		friction,
		{ 0.0f, -glassThickness * 0.5f });

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

/// Draws ImGui controls
void drawGui(nph::WorldDrawSettings& drawSettings)
{
	ImGui::SetNextWindowPos(ImVec2(10.0f, 10.0f), ImGuiCond_Once);
	ImGui::SetNextWindowSize(ImVec2(300.0f, 500.0f), ImGuiCond_Once);

	ImGui::Begin("Settings");
	ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.5f);

	if (ImGui::CollapsingHeader(
		"Visualization",
		ImGuiTreeNodeFlags_DefaultOpen))
	{
		ImGui::Checkbox(
			"Body Velocities",
			&drawSettings.bodyVelocities);
		ImGui::Checkbox(
			"AABBs",
			&drawSettings.aabbs);

		ImGui::SliderFloat(
			"Arrows Size",
			&drawSettings.arrowsTipSize,
			0.1f,
			1.0f);
	}

	ImGui::End();
}

} // anonymous namespace

/// The application entry point
int main()
{
	try
	{
		nph::World world(MAX_BODIES, { 0.0f, -GRAVITY }, 15);
		const nph::Vec2 glassSize{ GRAVITY * 0.5f, GRAVITY };
		const nph::Vec2 boxSize{ glassSize.x / 8.0f, glassSize.x / 16.0f };
		const float boxMass = boxSize.x * boxSize.y * 1000.0f;
		const float friction = 0.5f;

		createGlass(
			world,
			glassSize,
			GRAVITY * 0.05f,
			friction);

		nph::Visualization* visualization = nph::Visualization::getInstance();
		if (visualization == nullptr)
		{
			return -1;
		}
		visualization->setCameraZoom(int(glassSize.x * 2.0f));
		visualization->setCameraPan(nph::Vec2(0.0f, glassSize.y * 0.5f));

		nph::WorldDrawSettings drawSettings;
		while (visualization->isRunning())
		{
			visualization->startFrame();
			visualization->drawWorld(world, drawSettings);
			drawGui(drawSettings);
			bool overImgui = ImGui::IsWindowHovered(ImGuiHoveredFlags_AnyWindow);
			visualization->endFrame();

			if (visualization->getInput().leftMouseDown && !overImgui)
			{
				world.addBody(
					boxSize,
					boxMass,
					friction,
					visualization->getCursorPositionWorld(),
					0.5f);
			}

			world.doStep(SIMULATION_STEP);
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
