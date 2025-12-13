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

/// RGB color
struct Color
{
	float r{ 0.0f };
	float g{ 0.0f };
	float b{ 0.0f };
};

/// Window state
int gWindowWidth = (1920 * 3) / 4;
int gWindowHeight = (1080 * 3) / 4;

/// Camera state
struct Camera
{
	int zoom = 1;
	Vec2 pan{ 0.0f, 0.0f };
	Vec2 startPan{ 0.0f, 0.0f };
	Vec2 startPos{ 0.0f, 0.0f };
	bool panning = false;
} gCamera;

/// Input state
Visualization::Input gInput;

/// Convert cursor position to the world coordinates
Vec2 cursorToWorld(const Vec2& pos)
{
	const float aspect = float(gWindowWidth) / float(gWindowHeight);
	const float viewWidth = gCamera.zoom * std::min(1.0f, aspect);
	const float viewHeight = gCamera.zoom / std::max(1.0f, aspect);

	const float worldX = (pos.x / gWindowWidth) * (2.0f * viewWidth) - viewWidth + gCamera.pan.x;
	const float worldY = -((pos.y / gWindowHeight) * (2.0f * viewHeight) - viewHeight) + gCamera.pan.y;

	return { worldX, worldY };
}

/// Callback for GLFW errors
void glfwErrorCallback(
	int error,
	const char* description)
{
	logError("GLFW error ", error, ": ", description);
}

/// Update the OpenGL projection matrix based on the camera state
void updateProjectionMatrix()
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	const float aspect =
		static_cast<float>(gWindowWidth) /
		static_cast<float>(gWindowHeight);

	const float viewWidth = gCamera.zoom * std::min(1.0f, aspect);
	const float viewHeight = gCamera.zoom / std::max(1.0f, aspect);

	glOrtho(
		-viewWidth + gCamera.pan.x, // left
		viewWidth + gCamera.pan.x,  // right
		-viewHeight + gCamera.pan.y, // bottom
		viewHeight + gCamera.pan.y, // top
		-1.0, // near plane
		1.0   // far plane
	);
}

/// Callback for GLFW window resize events
void windowResizeCallback(
	GLFWwindow* window,
	int width,
	int height)
{
	NPH_UNUSED(window);
	gWindowWidth = width;
	gWindowHeight = height;
	glViewport(0, 0, width, height);
	updateProjectionMatrix();
}

/// Callback for GLFW cursor position events
void cursorPosCallback(
	GLFWwindow* window,
	double xpos,
	double ypos)
{
	NPH_UNUSED(window);
	if (gCamera.panning)
	{
		const Vec2 currentPos(
			static_cast<float>(xpos),
			static_cast<float>(ypos));
		gCamera.pan =
			gCamera.startPan +
			cursorToWorld(gCamera.startPos) - cursorToWorld(currentPos);
		updateProjectionMatrix();
	}
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
		gInput.leftMouseDown = (action == GLFW_PRESS);
		break;

	case GLFW_MOUSE_BUTTON_RIGHT:
		gInput.rightMouseDown = (action == GLFW_PRESS);
		break;

	case GLFW_MOUSE_BUTTON_MIDDLE:
		if (action == GLFW_PRESS)
		{
			gCamera.panning = true;
			gCamera.startPan = gCamera.pan;
			double x;
			double y;
			glfwGetCursorPos(window, &x, &y);
			gCamera.startPos = {
				static_cast<float>(x),
				static_cast<float>(y) };
		}
		else
		{
			gCamera.panning = false;
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
	constexpr int MIN_ZOOM = 1;
	constexpr int ZOOM_SPEED = 2;

	NPH_UNUSED(window);
	NPH_UNUSED(xoffset);
	gCamera.zoom = std::max(
		MIN_ZOOM,
		gCamera.zoom - static_cast<int>(yoffset * ZOOM_SPEED));

	updateProjectionMatrix();
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
	// Enable vsync
	glfwSwapInterval(1);

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

/// Draws an arrow
void drawArrow(
	const Vec2& start,
	const Vec2& end,
	const Color& color,
	float headSize = 0.2f)
{
	assert(headSize > 0.0f);

	const Vec2 dir = end - start;
	const Vec2 orthoLeft = getLeftOrthoVec(dir);
	const Vec2 leftArrowHead = end - headSize * dir + 0.5f * headSize * orthoLeft;
	const Vec2 rightArrowHead = end - headSize * dir - 0.5f * headSize * orthoLeft;

	glColor3f(color.r, color.g, color.b);
	glBegin(GL_LINES);
	// Line
	glVertex2f(start.x, start.y);
	glVertex2f(end.x, end.y);
	// Arrowhead
	glVertex2f(leftArrowHead.x, leftArrowHead.y);
	glVertex2f(rightArrowHead.x, rightArrowHead.y);
	glVertex2f(end.x, end.y);
	glEnd();
}

/// Draw a body
void drawBody(const Body& body)
{
	const Mat22& rot = body.rotation.getMat();
	const Vec2& pos = body.position;
	const Vec2& hs = body.halfSize;

	const Vec2 v1 = pos + rot * Vec2(-hs.x, -hs.y);
	const Vec2 v2 = pos + rot * Vec2( hs.x, -hs.y);
	const Vec2 v3 = pos + rot * Vec2( hs.x,  hs.y);
	const Vec2 v4 = pos + rot * Vec2(-hs.x,  hs.y);

	glColor3f(0.8f, 0.8f, 0.8f);
	glBegin(GL_LINE_LOOP);
	glVertex2f(v1.x, v1.y);
	glVertex2f(v2.x, v2.y);
	glVertex2f(v3.x, v3.y);
	glVertex2f(v4.x, v4.y);
	glEnd();
}

} // anonymous namespace

Visualization::Visualization()
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

	updateProjectionMatrix();
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

const Visualization::Input& Visualization::getInput() const
{
	return gInput;
}

const Vec2& Visualization::getCameraPan() const
{
	return gCamera.pan;
}

void Visualization::setCameraPan(const Vec2& pan)
{
	gCamera.pan = pan;
	updateProjectionMatrix();
}

int Visualization::getCameraZoom() const
{
	return gCamera.zoom;
}

void Visualization::setCameraZoom(int zoom)
{
	gCamera.zoom = zoom;
	updateProjectionMatrix();
}

Vec2 Visualization::getCursorPositionWorld() const
{
	double x;
	double y;
	glfwGetCursorPos(mWindow, &x, &y);
	return cursorToWorld({ static_cast<float>(x), static_cast<float>(y) });
}

void Visualization::drawWorld(
	const World& world,
	const WorldDrawSettings& settings)
{
	for (const Body& body : world.getBodies())
	{
		drawBody(body);
		if (settings.bodyVelocities)
		{
			drawArrow(
				body.position,
				body.position + body.linearVelocity,
				{0.0f, 0.0f, 1.0f});
		}
	}
}

} // namespace nph
