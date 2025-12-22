// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025 Dmitry Sapelnikov
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

	/// Draw body frames (coordinate axes)
	bool bodyFrames{ false };

	/// Size of the body frames
	float bodyFrameSize{ 0.2f };

	/// Draw contact points
	bool contacts{ false };

	/// Size of contact points
	float contactSize{ 5.0f };

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

	/// Sets the camera position
	void setCameraPosition(const Vec3& position);

	/// Sets the camera target
	void setCameraTarget(const Vec3& target);

	/// Sets the window size
	void setWindowSize(const Vec2& size);

	/// Updates the camera based on the input state
	void updateCamera();

	/// Sets the clear color
	void setClearColor(float r, float g, float b) const;

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
			drawAabb(aabb);
		}
	}

	for (const Body<2>&body : world.getBodies())
	{
		drawBody(body);
		if (settings.bodyVelocities)
		{
			drawArrow(
				body.position,
				body.position + body.linearVelocity,
				settings.bodyVelocityArrowSize,
				{ 1.0f, 0.0f, 1.0f });
		}

		if (settings.bodyFrames)
		{
			drawFrame(
				body.position,
				body.rotation.getMat(),
				settings.bodyFrameSize);
		}
	}

	if (settings.contacts)
	{
		drawContacts(world, settings.contactSize);
	}
}

// End of nph namespace
}
