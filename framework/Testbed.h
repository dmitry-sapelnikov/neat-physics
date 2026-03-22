// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025-2026 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#include <chrono>
#include "Core.h"
#include "Gui.h"
#include "Visualization.h"

namespace nph
{

/// The testbed application class
template <uint16_t D>
class Testbed
{
public:
	/// World setup callback type
	using WorldSetupCallback = std::function<std::unique_ptr<World<D>>()>;

	/// Visualization setup callback type
	using VisualizationSetupCallback = std::function<void(Visualization&)>;

	/// Scene creation callback type
	using SceneCreationCallback = std::function<void(World<D>&)>;

	/// Gui draw callback type
	using GuiDrawCallback = std::function<void()>;

	/// World interaction callback type
	using WorldCallback = std::function<void(World<D>& world)>;

	/// Constructor
	Testbed(WorldSetupCallback worldSetup) :
		mWorldSetupCallback(std::move(worldSetup))
	{
	}

	/// Runs the testbed application
	/// \return exit code
	[[nodiscard]] int run(
		SimulationControl simulationControl = {},
		WorldDrawSettings drawSettings = {}) noexcept;

	/// Sets the scene creation callback
	void setSceneCreationCallback(
		SceneCreationCallback callback) noexcept
	{
		mSceneCreationCallback = std::move(callback);
	}

	/// Sets the visualization setup callback
	void setVisualizationSetupCallback(
		VisualizationSetupCallback callback) noexcept
	{
		mVisualizationSetupCallback = std::move(callback);
	}

	/// Sets the gui draw callback
	void setGuiDrawCallback(
		GuiDrawCallback callback) noexcept
	{
		mGuiDrawCallback = std::move(callback);
	}

	/// Sets the left mouse click callback
	void setLeftMouseClickCallback(
		WorldCallback callback) noexcept
	{
		mLeftMouseClickCallback = std::move(callback);
	}

	/// Sets the right mouse down callback
	void setRightMouseDownCallback(
		WorldCallback callback) noexcept
	{
		mRightMouseDownCallback = std::move(callback);
	}

private:
	/// Draws ImGui controls
	void drawGui(
		float lastPhysicsStepTime,
		nph::WorldDrawSettings& drawSettings,
		SimulationControl& simulationControl);

	/// Do physics step(s)
	void doPhysicsSteps(const SimulationControl& simulationControl);

	/// The physics world
	std::unique_ptr<World<D>> mWorld;

	/// World setup callback
	WorldSetupCallback mWorldSetupCallback;

	/// Visualization setup callback
	VisualizationSetupCallback mVisualizationSetupCallback;

	/// Scene creation callback
	SceneCreationCallback mSceneCreationCallback;

	/// Gui draw callback
	GuiDrawCallback mGuiDrawCallback;

	/// Left mouse click callback
	WorldCallback mLeftMouseClickCallback;

	/// Right mouse down callback
	WorldCallback mRightMouseDownCallback;

	/// Last step time
	std::chrono::steady_clock::time_point mLastFrameTime;

	/// Delta time
	double mDeltaTime = 0.0f;

	/// Last physics step time
	float mLastPhyicsStepTime = 0.0f;
};

template <uint16_t D>
[[nodiscard]] int Testbed<D>::run(
	SimulationControl simulationControl,
	WorldDrawSettings drawSettings) noexcept
{
	try
	{
		mWorld = mWorldSetupCallback();
		if (mWorld.get() == nullptr)
		{
			nph::logError("World setup callback returned nullptr.");
			return -1;
		}

		nph::Visualization* visualization = nph::Visualization::getInstance();
		if (visualization == nullptr)
		{
			return -1;
		}

		if (mVisualizationSetupCallback)
		{
			mVisualizationSetupCallback(*visualization);
		}

		mLastFrameTime = std::chrono::high_resolution_clock::now();
		while (visualization->isRunning())
		{
			auto frameStartTime = std::chrono::high_resolution_clock::now();
			if (simulationControl.resetWorld)
			{
				mWorld->clear();
				if (mSceneCreationCallback)
				{
					mSceneCreationCallback(*mWorld);
				}
				simulationControl.resetWorld = false;
			}

			if (visualization->getMouseInput().leftMouseClicked &&
				mLeftMouseClickCallback != nullptr)
			{
				mLeftMouseClickCallback(*mWorld);
			}

			if (visualization->getMouseInput().rightMouseDown &&
				mRightMouseDownCallback != nullptr)
			{
				mRightMouseDownCallback(*mWorld);
			}

			visualization->startFrame();
			visualization->drawWorld(*mWorld, drawSettings);
			drawGui(
				mLastPhyicsStepTime,
				drawSettings,
				simulationControl);
			visualization->endFrame();

			const bool overImGui =
				ImGui::IsAnyItemHovered() ||
				ImGui::IsAnyItemActive() ||
				ImGui::IsAnyItemFocused() ||
				ImGui::IsWindowHovered(ImGuiHoveredFlags_AnyWindow);

			visualization->setCameraControllerEnabled(!overImGui);
			visualization->setVSyncEnabled(simulationControl.vSync);

			mWorld->getSettings().solverSettings.setVelocityIterations(
				uint32_t(simulationControl.velocityIterations));

			mWorld->getSettings().solverSettings.setPositionIterations(
				uint32_t(simulationControl.positionIterations));

			doPhysicsSteps(simulationControl);
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

template <uint16_t D>
void Testbed<D>::doPhysicsSteps(const SimulationControl& simulationControl)
{
	/// The max number of steps per frame to 
	/// prevent the death spiral of performance
	static constexpr double MAX_STEPS_PER_FRAME = 5;

	if (!(simulationControl.simulationRunning || simulationControl.doSimulationStep))
	{
		mLastFrameTime = std::chrono::high_resolution_clock::now();
		mDeltaTime = 0.0f;
	}

	const double timeStep = 1.0 / static_cast<double>(simulationControl.timeStepFrequency);
	if (simulationControl.simulationRunning)
	{
		if (simulationControl.fpsIndependentPhysics)
		{
			auto curTime = std::chrono::high_resolution_clock::now();
			mDeltaTime += std::chrono::duration<double>(curTime - mLastFrameTime).count();
			mDeltaTime = std::min(mDeltaTime, timeStep * MAX_STEPS_PER_FRAME);
			mLastFrameTime = curTime;
		}
		else
		{
			mDeltaTime = timeStep;
		}
	}
	else
	{
		mDeltaTime = simulationControl.doSimulationStep ? timeStep : 0.0f;
	}

	uint32_t physicsSteps = 0;
	const auto tic = std::chrono::high_resolution_clock::now();
	while (mDeltaTime >= timeStep)
	{
		mWorld->doStep(static_cast<float>(timeStep));
		mDeltaTime -= timeStep;
		++physicsSteps;
	}
	const auto toc = std::chrono::high_resolution_clock::now();
	if (physicsSteps > 0)
	{
		mLastPhyicsStepTime = std::chrono::duration<float>(toc - tic).count() / physicsSteps;
	}
}

template <uint16_t D>
void Testbed<D>::drawGui(
	float lastPhysicsStepTime,
	nph::WorldDrawSettings& drawSettings,
	SimulationControl& simulationControl)
{
	assert(mWorld.get() != nullptr);

	ImGui::SetNextWindowPos(ImVec2(10.0f, 10.0f), ImGuiCond_Once);
	ImGui::SetNextWindowSize(ImVec2(400.0f, 700.0f), ImGuiCond_Once);

	ImGui::Begin("Settings", nullptr, ImGuiWindowFlags_NoCollapse);
	ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.4f);

	drawVisualizationTab(drawSettings);
	drawStatsTab(*mWorld, lastPhysicsStepTime);
	drawSimulationControlTab(simulationControl);

	if (mGuiDrawCallback)
	{
		mGuiDrawCallback();
	}

	ImGui::PopItemWidth();
	ImGui::End();
}

} // namespace nph
