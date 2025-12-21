// A minimalistic 2D physics engine (https://github.com/dmitry-sapelnikov/neat-physics)
// SPDX-FileCopyrightText: 2025 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

// Includes
#include "glad/glad.h"
#include "GLFW/glfw3.h"
#include "imgui/imgui.h"

#include "neat_physics/math/Vec2.h"

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
	/// Input state
	struct Input
	{
		/// Left mouse down flag
		bool leftMouseDown = false;

		/// Left mouse clicked flag
		bool leftMouseClicked = false;

		/// Right mouse down flag
		bool rightMouseDown = false;
	};

	/// Visualization instance getter
	/// \return nullptr if initialization failed
	static Visualization* getInstance();

	/// Checks if the visualization is running
	bool isRunning() const;

	/// Start a new frame
	void startFrame();

	/// End the current frame
	void endFrame();

	/// Returns the input state
	const Input& getInput() const;

	/// Gets the camera pan
	const Vec2& getCameraPan() const;

	/// Sets the camera pan
	void setCameraPan(const Vec2& pan);

	/// Gets the camera zoom
	int getCameraZoom() const;

	/// Sets the camera zoom
	void setCameraZoom(int zoom);

	/// Returns the cursor position in world coordinates
	[[nodiscard]] Vec2 getCursorPositionWorld() const;

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
	void drawWorld(
		const World<2>& world,
		const WorldDrawSettings& settings);

private:
	// Singleton rule of five
	Visualization();
	~Visualization();
	Visualization(const Visualization&) = delete;
	Visualization& operator=(const Visualization&) = delete;
	Visualization(Visualization&&) = delete;

	/// GLFW window pointer
	GLFWwindow* mWindow{ nullptr };

	/// VSync enabled flag
	bool mVSyncEnabled{ false };
};

// End of nph namespace
}
