#include "Visualization.h"

#include "imgui/backends/imgui_impl_glfw.h"
#include "imgui/backends/imgui_impl_opengl2.h"

#include "Core.h"

namespace nph
{

// Local functions
namespace
{

/// Window dimensions
constexpr int WINDOW_WIDTH = 1024;
constexpr int WINDOW_HEIGHT = 768;

/// Callback for GLFW errors
void glfwErrorCallback(
	int error,
	const char* description)
{
	logError("GLFW error ", error, ": ", description);
}

/// Callback for GLFW keyboard events
void keyboardCallback(
	GLFWwindow* window,
	int key,
	int scanCode,
	int action,
	int mods)
{
	UNUSED(window);
	UNUSED(scanCode);
	UNUSED(mods);

	if (action != GLFW_PRESS)
	{
		return;
	}

	switch (key)
	{
	case GLFW_KEY_ESCAPE:
		// Quit
		glfwSetWindowShouldClose(window, true);
		break;
	}
}

/// Initialize GLFW and load OpenGL functions
bool initGlfw()
{
	glfwSetErrorCallback(glfwErrorCallback);
	if (glfwInit() == 0)
	{
		logError("GLFW initialization failed.");
		return false;
	}
	return true;
}

/// Create the main application window
GLFWwindow* createWindow(int width, int height)
{
	GLFWwindow* result = glfwCreateWindow(
		width,
		height,
		"Neat Physics",
		nullptr,
		nullptr);

	if (result == nullptr)
	{
		logError("Failed to open GLFW mainWindow.");
		glfwTerminate();
		return nullptr;
	}

	glfwSetKeyCallback(result, keyboardCallback);
	glfwMakeContextCurrent(result);
	// Enable vsync
	glfwSwapInterval(1);
	return result;
}

/**
	Inits OpenGL using glad
	\note this must be called after creating an OpenGL context
*/
bool initOpenGL()
{
	if (!gladLoadGL())
	{
		logError("Failed to load OpenGL functions using glad.");
		return false;
	}
	glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
	return true;
}

/// Initialize ImGui
bool initImgui(GLFWwindow& result)
{
	ImGuiContext* context = ImGui::CreateContext();
	if (context == nullptr)
	{
		logError("Failed to create ImGui context.");
		return false;
	}

	if (!ImGui_ImplGlfw_InitForOpenGL(&result, true))
	{
		logError("Failed to initialize ImGui_ImplGlfw.");
		return false;
	}

	if (!ImGui_ImplOpenGL2_Init())
	{
		logError("Failed to initialize ImGui_ImplOpenGL2.");
		return false;
	}

	float xscale{ 0.0 };
	float yscale{ 0.0 };
	glfwGetWindowContentScale(&result, &xscale, &yscale);

	ImGuiIO& io = ImGui::GetIO();
	io.FontGlobalScale = xscale;
	io.IniFilename = nullptr;

	return true;
}

}

// Global classes
Visualization::Visualization()
{
	if (!initGlfw())
	{
		return;
	}

	GLFWwindow* result = createWindow(WINDOW_WIDTH, WINDOW_HEIGHT);

	if (result == nullptr)
	{
		return;
	}

	if (!initOpenGL())
	{
		return;
	}

	if (!initImgui(*result))
	{
		return;
	}

	mWindow = result;
}

Visualization* Visualization::getInstance()
{
	static Visualization instance;
	return instance.mWindow != nullptr ? &instance : nullptr;
}

Visualization::~Visualization()
{
	if (mWindow != nullptr)
	{
		ImGui_ImplOpenGL2_Shutdown();
		ImGui_ImplGlfw_Shutdown();
		ImGui::DestroyContext();
	}
	// It is safe to call glfwTerminate even if glfwInit failed
	glfwTerminate();
}

bool Visualization::isRunning() const
{
	return !glfwWindowShouldClose(mWindow);
}

void Visualization::startFrame()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	ImGui_ImplOpenGL2_NewFrame();
	ImGui_ImplGlfw_NewFrame();
	ImGui::NewFrame();
}

void Visualization::endFrame()
{
	ImGui::Render();
	ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());

	glfwPollEvents();
	glfwSwapBuffers(mWindow);
}

// End of nph namespace
}
