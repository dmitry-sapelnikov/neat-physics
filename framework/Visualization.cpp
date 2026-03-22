// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025-2026 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

// Includes
#include "Visualization.h"
#include "imgui/backends/imgui_impl_glfw.h"
#include "imgui/backends/imgui_impl_opengl2.h"
#include "Core.h"
#include "neat_physics/World.h"

namespace nph
{

namespace
{

/// Initial window size
int gWindowWidth = (1920 * 3) / 4;
int gWindowHeight = (1080 * 3) / 4;

/// Input state
MouseInput gMouseInput;

/// Callback for GLFW errors
void glfwErrorCallback(
	int error,
	const char* description)
{
	logError("GLFW error ", error, ": ", description);
}


/// Callback for GLFW window resize events
void windowResizeCallback(
	GLFWwindow* window,
	int width,
	int height)
{
	NPH_UNUSED(window);

	assert(width >= 0);
	assert(height >= 0);
	if (width == 0 || height == 0)
	{
		return;
	}

	glViewport(0, 0, width, height);
	Visualization::getInstanceRef().setWindowSize({
		static_cast<float>(width),
		static_cast<float>(height) });
}

/// Callback for GLFW cursor position events
void cursorPosCallback(
	GLFWwindow* window,
	double xpos,
	double ypos)
{
	NPH_UNUSED(window);
	gMouseInput.position = {
		static_cast<float>(xpos),
		static_cast<float>(ypos)
	};
	Visualization::getInstanceRef().updateCamera();
}

/// Callback for GLFW mouse button events
void mouseButtonCallback(
	GLFWwindow* window,
	int button,
	int action,
	int mods)
{
	NPH_UNUSED(window);
	NPH_UNUSED(mods);

	switch (button)
	{
	case GLFW_MOUSE_BUTTON_LEFT:
	{
		gMouseInput.leftMouseDown = (action == GLFW_PRESS);
		if (action == GLFW_PRESS)
		{
			gMouseInput.leftMouseClicked = true;
		}
		Visualization::getInstanceRef().updateCamera();
	}
	break;

	case GLFW_MOUSE_BUTTON_RIGHT:
	{
		gMouseInput.rightMouseDown = (action == GLFW_PRESS);
		Visualization::getInstanceRef().updateCamera();
	}
	break;

	case GLFW_MOUSE_BUTTON_MIDDLE:
	{
		gMouseInput.middleMouseDown = (action == GLFW_PRESS);
		Visualization::getInstanceRef().updateCamera();
	}
	break;

	}
}

/// Callback for GLFW scroll events
void scrollCallback(
	GLFWwindow* window,
	double xoffset,
	double yoffset)
{
	gMouseInput.mouseWheel = yoffset;
	Visualization::getInstanceRef().updateCamera();
	gMouseInput.mouseWheel = 0.0;
}

/// Callback for GLFW keyboard events
void keyboardCallback(
	GLFWwindow* window,
	int key,
	int scanCode,
	int action,
	int mods)
{
	NPH_UNUSED(window);
	NPH_UNUSED(scanCode);
	NPH_UNUSED(mods);

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
GLFWwindow* createWindow()
{
	// Request multisampling with 4 samples
	glfwWindowHint(GLFW_SAMPLES, 4);

	GLFWwindow* result = glfwCreateWindow(
		gWindowWidth,
		gWindowHeight,
		"Neat Physics",
		nullptr,
		nullptr);

	if (result == nullptr)
	{
		logError("Failed to open GLFW mainWindow.");
		glfwTerminate();
		return nullptr;
	}

	glfwSetWindowSizeCallback(result, windowResizeCallback);
	glfwSetMouseButtonCallback(result, mouseButtonCallback);
	glfwSetCursorPosCallback(result, cursorPosCallback);
	glfwSetScrollCallback(result, scrollCallback);
	glfwSetKeyCallback(result, keyboardCallback);
	glfwMakeContextCurrent(result);
	return result;
}

/// Inits OpenGL using glad
/// \note this must be called after creating an OpenGL context
bool initOpenGL()
{
	if (!gladLoadGL())
	{
		logError("Failed to load OpenGL functions using glad.");
		return false;
	}

	// Enable alpha blending for body fill
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	// Enable multisampling
	glEnable(GL_MULTISAMPLE);

	// Enable z-test
	glEnable(GL_DEPTH_TEST);

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

	float xScale{ 0.0f };
	float yScale{ 0.0f };
	glfwGetWindowContentScale(&result, &xScale, &yScale);

	ImGuiIO& io = ImGui::GetIO();
	io.FontGlobalScale = xScale;
	// Disable creation of the .ini file
	io.IniFilename = nullptr;

	return true;
}

} // anonymous namespace

Visualization::Visualization() :
	mCamera(
		{ 0.0f, 0.0f, -10.0f },
		{ 0.0f, 0.0f, 0.0f },
		{ 0.0f, 1.0f, 0.0f },
		45.0f,
		0.1f,
		1000.0f,
		{ static_cast<float>(gWindowWidth),
		  static_cast<float>(gWindowHeight) }),

	mCameraController(
		mCamera,
		0.0f,
		75.0f,
		1.0f,
		1000.0f)
{
	if (!initGlfw())
		return;

	GLFWwindow* result = createWindow();
	if (result == nullptr)
		return;

	if (!initOpenGL())
		return;

	if (!initImgui(*result))
		return;

	setVSyncEnabled(true);
	setClearColor(0.0f, 0.0f, 20.0f / 255.0f);
	updateViewProjectionMatrix();
	mWindow = result;
}

Visualization* Visualization::getInstance()
{
	static Visualization instance;
	return instance.mWindow != nullptr ? &instance : nullptr;
}

Visualization& Visualization::getInstanceRef()
{
	Visualization* result = getInstance();
	assert(result != nullptr);
	return *result;
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

	gMouseInput.leftMouseClicked = false;
	glfwPollEvents();
	glfwSwapBuffers(mWindow);
}

const MouseInput& Visualization::getMouseInput() const
{
	return gMouseInput;
}

void Visualization::setCameraPosition(const Vec3& position)
{
	mCameraController.setCameraPosition(position);
	updateViewProjectionMatrix();
}

void Visualization::setCameraTarget(const Vec3& target)
{
	mCameraController.setCameraTarget(target);
	updateViewProjectionMatrix();
}

void Visualization::setRotationSpeed(float speed)
{
	mCameraController.setRotationSpeed(speed);
}

void Visualization::setZoomSpeed(float speed)
{
	mCameraController.setZoomSpeed(speed);
}

void Visualization::setClearColor(float r, float g, float b) const
{
	glClearColor(r, g, b, 1.0f);
}

void Visualization::setDepthTest(bool enabled) const
{
	if (enabled)
	{
		glEnable(GL_DEPTH_TEST);
	}
	else
	{
		glDisable(GL_DEPTH_TEST);
	}
}

void Visualization::setVSyncEnabled(bool enabled)
{
	if (enabled != mVSyncEnabled)
	{
		mVSyncEnabled = enabled;
		glfwSwapInterval(mVSyncEnabled);
	}
}

void Visualization::setWindowSize(const Vec2& size)
{
	assert(size.x > 0.0f);
	assert(size.y > 0.0f);
	mCamera.getProjection().setWindowSize(size);
	updateViewProjectionMatrix();
}

void Visualization::updateCamera()
{
	if (mCameraControllerEnabled &&
		mCameraController.update(getMouseInput()))
	{
		updateViewProjectionMatrix();
	}
}

void Visualization::updateViewProjectionMatrix()
{
	glMatrixMode(GL_MODELVIEW);
	glLoadMatrixf(mCamera.getView().getMatrix().data());

	glMatrixMode(GL_PROJECTION);
	glLoadMatrixf(mCamera.getProjection().getMatrix().data());
}

} // namespace nph
