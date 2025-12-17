// A minimalistic 2D physics engine (https://github.com/dmitry-sapelnikov/neat-physics)
// SPDX-FileCopyrightText: 2025 Dmitry Sapelnikov
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

/// Converts cursor position to the world coordinates
Vec2 cursorToWorld(const Vec2& pos)
{
	const float aspect = float(gWindowWidth) / float(gWindowHeight);
	const float viewWidth = gCamera.zoom * std::min(1.0f, aspect);
	const float viewHeight = gCamera.zoom / std::max(1.0f, aspect);

	const float worldX = gCamera.pan.x +
		(pos.x / gWindowWidth) * (2.0f * viewWidth) - viewWidth;

	const float worldY = gCamera.pan.y -
		((pos.y / gWindowHeight) * (2.0f * viewHeight) - viewHeight);

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
		viewWidth + gCamera.pan.x, // right
		-viewHeight + gCamera.pan.y, // bottom
		viewHeight + gCamera.pan.y, // top
		-1.0, // near plane
		 1.0 // far plane
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
	{
		gInput.leftMouseDown = (action == GLFW_PRESS);
		if (action == GLFW_PRESS)
		{
			gInput.leftMouseClicked = true;
		}
	}
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
	glClearColor(0.0f, 0.0f, 20.0f / 255.0f, 1.0f);

	// Enable alpha blending for body fill
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

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
	float tipSize,
	const Color& color)
{
	/// The ratio between the tip height and side length
	static constexpr float TIP_SIDE_FACTOR = 0.3f;
	assert(tipSize > 0.0f);

	const Vec2 dir = end - start;
	const Vec2 dirNorm = dir.getNormalized();
	const Vec2 orthoLeft = getLeftOrthoVec(dirNorm);
	const Vec2 tipEnd = end + tipSize * dirNorm;
	const Vec2 leftArrowHead =  end + TIP_SIDE_FACTOR * tipSize * orthoLeft;
	const Vec2 rightArrowHead = end - TIP_SIDE_FACTOR * tipSize * orthoLeft;

	glColor3f(color.r, color.g, color.b);
	glBegin(GL_LINES);
	// Line
	glVertex2f(start.x, start.y);
	glVertex2f(end.x, end.y);
	// Arrowhead
	glVertex2f(leftArrowHead.x, leftArrowHead.y);
	glVertex2f(rightArrowHead.x, rightArrowHead.y);
	glVertex2f(tipEnd.x, tipEnd.y);
	glVertex2f(leftArrowHead.x, leftArrowHead.y);
	glVertex2f(tipEnd.x, tipEnd.y);
	glVertex2f(rightArrowHead.x, rightArrowHead.y);
	glEnd();
}

/// Draws a body
void drawBody(const Body& body)
{
	const Mat22& rot = body.rotation.getMat();
	const Vec2& pos = body.position;
	const Vec2& hs = body.halfSize;

	const Vec2 v1 = pos + rot * Vec2(-hs.x, -hs.y);
	const Vec2 v2 = pos + rot * Vec2(hs.x, -hs.y);
	const Vec2 v3 = pos + rot * Vec2(hs.x, hs.y);
	const Vec2 v4 = pos + rot * Vec2(-hs.x, hs.y);

	glColor4f(1.0f, 1.0f, 0.9f, body.isStatic() ? 0.3f : 0.15f);
	glBegin(GL_TRIANGLE_FAN);
	glVertex2f(v1.x, v1.y);
	glVertex2f(v2.x, v2.y);
	glVertex2f(v3.x, v3.y);
	glVertex2f(v4.x, v4.y);
	glEnd();

	glColor3f(0.8f, 0.8f, 0.75f);
	glBegin(GL_LINE_LOOP);
	glVertex2f(v1.x, v1.y);
	glVertex2f(v2.x, v2.y);
	glVertex2f(v3.x, v3.y);
	glVertex2f(v4.x, v4.y);
	glEnd();
}

/// Draws an Aabb
void drawAabb(const Aabb& aabb)
{
	glColor3f(0.0f, 0.5f, 0.0f);
	glBegin(GL_LINE_LOOP);
	glVertex2f(aabb.min.x, aabb.min.y);
	glVertex2f(aabb.max.x, aabb.min.y);
	glVertex2f(aabb.max.x, aabb.max.y);
	glVertex2f(aabb.min.x, aabb.max.y);
	glEnd();
}

/// Draws a frame
void drawFrame(const Vec2& position, const Mat22& rotation, float size)
{
	const Vec2 xAxis = position + rotation * Vec2(size, 0.0f);
	const Vec2 yAxis = position + rotation * Vec2(0.0f, size);
	drawArrow(
		position,
		xAxis,
		size * 0.2f,
		{ 1.0f, 0.0f, 0.0f });

	drawArrow(
		position,
		yAxis,
		size * 0.2f,
		{ 0.0f, 1.0f, 0.0f });
}

/// Draw contact points
void drawContacts(const World& world, float pointSize)
{
	assert(pointSize > 0.0f);
	glPointSize(pointSize);
	glColor3f(1.0f, 0.0f, 0.0f);
	glBegin(GL_POINTS);
	for (const auto& pair : world.getContactSolver().getManifolds())
	{
		const ContactManifold& manifold = pair.second;
		const Body& bodyA = manifold.getBodyA();
		const Body& bodyB = manifold.getBodyB();

		for (uint32_t i = 0; i < manifold.getContactCount(); ++i)
		{
			const CollisionPoint& point = manifold.getContact(i).getPoint();
			const Vec2 point1 = bodyA.position + bodyA.rotation.getMat() * point.localPoints[0];
			const Vec2 point2 = bodyB.position + bodyB.rotation.getMat() * point.localPoints[1];

			// Draw contact point on body A
			glVertex2f(point1.x, point1.y);

			// Draw contact point on body B
			glVertex2f(point2.x, point2.y);
		}
	}
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

	setVSyncEnabled(true);
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

	gInput.leftMouseClicked = false;
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

void Visualization::setClearColor(float r, float g, float b) const
{
	glClearColor(r, g, b, 1.0f);
}

void Visualization::setVSyncEnabled(bool enabled)
{
	if (enabled != mVSyncEnabled)
	{
		mVSyncEnabled = enabled;
		glfwSwapInterval(mVSyncEnabled);
	}
}

void Visualization::drawWorld(
	const World& world,
	const WorldDrawSettings& settings)
{
	/// \note AABBs are drawn as they were at
	/// the beginning of the last simulation step,
	/// so they may not match the bodies' current positions
	if (settings.aabbs)
	{
		for (const Aabb& aabb : world.getCollision().getBroadPhase().getAabbs())
		{
			drawAabb(aabb);
		}
	}

	for (const Body& body : world.getBodies())
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

} // namespace nph
