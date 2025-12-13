#pragma once

// Includes
#include "glad/glad.h"
#include "GLFW/glfw3.h"
#include "imgui/imgui.h"

#include "neat_physics/math/Vec2.h"

namespace nph
{

// Global classes
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
