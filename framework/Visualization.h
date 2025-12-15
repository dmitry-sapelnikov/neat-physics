#pragma once

// Includes
#include "glad/glad.h"
#include "GLFW/glfw3.h"
#include "imgui/imgui.h"

#include "neat_physics/math/Vec2.h"

namespace nph
{

// Forward declarations
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
	float bodyVelocityArrowSize{ 0.2f };
};

/// Singleton class managing the visualization system
class Visualization
{
public:
	/// Input state
	struct Input
	{
		bool leftMouseDown = false;
		bool rightMouseDown = false;
	};

	/// Instance getter. Returns nullptr if initialization failed
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
	Vec2 getCursorPositionWorld() const;

	/// Sets the clear color
	void setClearColor(float r, float g, float b) const;

	/// Draws a physics world
	void drawWorld(
		const World& world,
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
};

// End of nph namespace
}
