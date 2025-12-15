#include <chrono>
#include "Core.h"
#include "Visualization.h"
#include "neat_physics/World.h"

namespace nph
{
namespace
{

/// Maximum number of bodies in the physics world
static constexpr uint32_t MAX_BODIES = 10000;

/// Gravity
static constexpr float GRAVITY = 10.0f;

/// Simulation step
static constexpr float SIMULATION_STEP = 1.0f / 60.0f;

/// Simulation control parameters
struct SimulationControl
{
	bool resetWorld{ true };
	float wallFriction{ 0.0f };
	float friction{ 0.0f };
	float boxDensity{ 200.0f };
	float boxSize{ 8.0f };
	float boxSideRatio{ 0.5f };
	float timeStepFrequency{ 50.0f };
	int velocityIterations{ 15 };
};

/// Creates a 'glass-shaped' container
void createGlass(
	nph::World& world,
	const nph::Vec2& glassSize,
	float glassThickness,
	float friction)
{
	// Create a static 'glass' made of 3 bodies: bottom and 2 sides
	const float bottomSize = glassSize.x * 100.0f;
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
		ImGui::IsAnyItemFocused() ||
		ImGui::IsAnyItemHovered() ||
		ImGui::IsWindowFocused(ImGuiFocusedFlags_AnyWindow) ||
		ImGui::IsWindowHovered(ImGuiHoveredFlags_AnyWindow);

	if (overImgui || !visualization->getInput().leftMouseDown)
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
	SimulationControl& simulationControl)
{
	ImGui::SetNextWindowPos(ImVec2(10.0f, 10.0f), ImGuiCond_Once);
	ImGui::SetNextWindowSize(ImVec2(400.0f, 500.0f), ImGuiCond_Once);

	ImGui::Begin("Settings", nullptr, ImGuiWindowFlags_NoCollapse);

	ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.4f);

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
	}

	if (ImGui::CollapsingHeader(
		"Simulation Control",
		ImGuiTreeNodeFlags_DefaultOpen))
	{
		ImGui::Indent(20.0f);

		// Add 'World' splitter
		if (ImGui::CollapsingHeader("World", ImGuiTreeNodeFlags_DefaultOpen))
		{
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

			ImGui::SliderFloat(
				"Wall Friction",
				&simulationControl.wallFriction,
				0.0f,
				1.0f,
				"%.2f");

			simulationControl.resetWorld = ImGui::Button("Reset World");
		}

		if (ImGui::CollapsingHeader(
			"New Boxes",
			ImGuiTreeNodeFlags_DefaultOpen))
		{
			ImGui::SliderFloat(
				"Friction",
				&simulationControl.friction,
				0.0f,
				1.0f,
				"%.2f");

			ImGui::SliderFloat(
				"Size",
				&simulationControl.boxSize,
				2.0f,
				20.0f,
				"1 / %.0f of glass size");

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
		const float friction = 0.5f;

		nph::World world(nph::MAX_BODIES, { 0.0f, -nph::GRAVITY }, 15);

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
					simulationControl.wallFriction);
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
				simulationControl);
			visualization->endFrame();

			world.setVelocityIterations(
				uint32_t(simulationControl.velocityIterations));

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
