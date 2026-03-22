// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025-2026 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

// Includes
#include "glad/glad.h"
#include "GLFW/glfw3.h"
#include "imgui/imgui.h"

#include "camera/Camera.h"
#include "camera/MouseCameraController.h"
#include "MouseInput.h"
#include "DrawFunctions.h"

namespace nph
{

// Forward declarations
template <uint16_t D>
class World;

/// Settings for world visualization
struct WorldDrawSettings
{
	/// Draw axis-aligned bounding boxes around bodies
	bool aabbs{ false };

	/// Draw the AABB tree nodes
	bool aabbTree{ false };

	/// Draw body frames (coordinate axes)
	bool bodyFrames{ false };

	/// Size of the body frames
	float bodyFrameSize{ 0.2f };

	/// Draw contact points
	bool contactPoints{ false };

	/// Draw contact areas
	bool contactAreas{ false };

	/// Draw contact frames
	bool contactFrames{ false };

	/// Size of contact points
	float contactPointSize{ 5.0f };

	/// Size of contact frames
	float contactFrameSize{ 0.1f };

	/// Draw body linear velocities
	bool bodyVelocities{ false };

	/// Size of body velocity arrows
	float bodyVelocityArrowSize{ 0.1f };
};

/// Singleton class managing the visualization system
class Visualization
{
public:
	/// Visualization instance getter
	/// \return nullptr if initialization failed
	static Visualization* getInstance();

	/// Visualization instance getter; asserts if the instance != nullptr
	static Visualization& getInstanceRef();

	/// Checks if the visualization is running
	bool isRunning() const;

	/// Start a new frame
	void startFrame();

	/// End the current frame
	void endFrame();

	/// Returns the input state
	const MouseInput& getMouseInput() const;

	/// Returns the camera (const version)
	const Camera& getCamera() const
	{
		return mCamera;
	}

	/// Enables/disables camera control
	void setCameraControllerEnabled(bool enabled)
	{
		mCameraControllerEnabled = enabled;
	}

	/// Sets the camera position
	void setCameraPosition(const Vec3& position);

	/// Sets the camera target
	void setCameraTarget(const Vec3& target);

	/// Sets the rotation speed
	void setRotationSpeed(float speed);

	/// Sets the zoom speed
	void setZoomSpeed(float speed);

	/// Sets the window size
	void setWindowSize(const Vec2& size);

	/// Updates the camera based on the input state
	void updateCamera();

	/// Sets the clear color
	void setClearColor(float r, float g, float b) const;

	/// Enable z-depth testing
	void setDepthTest(bool enabled) const;

	/// Returns if VSync is enabled
	[[nodiscard]] bool isVSyncEnabled() const
	{
		return mVSyncEnabled;
	}

	/// Set VSync enabled / disabled
	void setVSyncEnabled(bool enabled);

	/// Draws a physics world
	template <uint16_t D>
	void drawWorld(
		const World<D>& world,
		const WorldDrawSettings& settings);

private:
	// Singleton rule of five
	Visualization();
	~Visualization();
	Visualization(const Visualization&) = delete;
	Visualization& operator=(const Visualization&) = delete;
	Visualization(Visualization&&) = delete;

	/// Updates the view-projection matrix
	void updateViewProjectionMatrix();

	/// GLFW window pointer
	GLFWwindow* mWindow{ nullptr };

	/// VSync enabled flag
	bool mVSyncEnabled{ false };

	/// Camera
	Camera mCamera;

	/// Camera controller
	MouseCameraController mCameraController;

	/// Camera controller enabled flag
	bool mCameraControllerEnabled{ true };
};

template <uint16_t D>
void Visualization::drawWorld(
	const World<D>& world,
	const WorldDrawSettings& settings)
{
	/// \note AABBs are drawn as they were at
	/// the beginning of the last simulation step,
	/// so they may not match the bodies' current positions
	if (settings.aabbs)
	{
		for (const Aabb<D>& aabb :
			world.getCollision().getBroadPhase().getAabbs())
		{
			drawAabb(aabb, Color(0.0f, 0.5f, 0.0f));
		}
	}

	if (settings.aabbTree)
	{
		world.getCollision().getBroadPhase().getAabbTree().processAllNodes(
			[](const Aabb<D>& aabb, int32_t height)
			{
				drawAabb(aabb, getColorRamp(height / 10.0f));
			});
	}

	const auto& bodyStorage = world.getBodyStorage();
	auto bodyTransforms = bodyStorage.getComponent<ecs::TransformTag>();
	auto bodyHalfSizes = bodyStorage.getComponent<ecs::HalfSizeTag>();
	auto bodyVelocities = bodyStorage.getComponent<ecs::VelocityTag>();
	auto bodyMassProperties = bodyStorage.getComponent<ecs::MassPropertiesTag>();
	for (size_t i = 0; i < bodyStorage.size(); ++i)
	{
		const Transform<D>& bodyTransform = bodyTransforms[i];
		drawBody(
			bodyTransform,
			bodyHalfSizes[i],
			bodyMassProperties[i].isStatic());

		if (settings.bodyVelocities)
		{
			drawArrow(
				bodyTransform.position,
				bodyTransform.position + bodyVelocities[i].linear,
				settings.bodyVelocityArrowSize,
				{ 1.0f, 0.0f, 1.0f });
		}

		if (settings.bodyFrames)
		{
			drawFrame(bodyTransform, settings.bodyFrameSize);
		}
	}

	if (settings.contactPoints)
	{
		drawContactPoints(world, settings.contactPointSize);
	}

	if (settings.contactAreas)
	{
		drawContactAreas(world);
	}

	if (settings.contactFrames)
	{
		drawContactFrames(world, settings.contactFrameSize);
	}
}

// End of nph namespace
}
