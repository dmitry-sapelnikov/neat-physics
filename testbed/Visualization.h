#pragma once

// Includes
#include "glad/glad.h"
#include "GLFW/glfw3.h"
#include "imgui/imgui.h"

namespace nph
{

// Global classes
/// Singleton class managing the visualization system
class Visualization
{
public:
	/// Instance getter. Returns nullptr if initialization failed
	static Visualization* getInstance();

	/// Checks if the visualization is running
	bool isRunning() const;

	/// Start a new frame
	void startFrame();

	/// End the current frame
	void endFrame();

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
