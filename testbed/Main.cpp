#include "Core.h"
#include "Visualization.h"
#include "neat_physics/World.h"

using namespace nph;
namespace
{

/// Maximum number of bodies in the physics world
static constexpr uint32_t MAX_BODIES = 1000;

/// Gravity
static constexpr float GRAVITY = 10.0f;

/// Creates a 'glass-shaped' container
void createGlass(
	nph::World& world,
	const nph::Vec2& glassSize,
	float glassThickness,
	float friction)
{
	// Create a static 'glass' made of 3 bodies: bottom and 2 sides
	world.addBody(
		{ glassSize.x + 2.0f * glassThickness, glassThickness },
		0.0f,
		friction,
		{ 0.0f, -glassThickness * 0.5f });

	world.addBody(
		{ glassThickness, glassSize.y },
		0.0f,
		friction,
		{ -(glassSize.x + glassThickness) * 0.5f, 0.5f * glassSize.y });

	world.addBody(
		{ glassThickness, glassSize.y },
		0.0f,
		friction,
		{ (glassSize.x + glassThickness) * 0.5f, 0.5f * glassSize.y });
}

} // anonymous namespace

/// The application entry point
int main()
{
	try
	{
		nph::World world(MAX_BODIES);
		const nph::Vec2 glassSize{ GRAVITY * 0.5f, GRAVITY };
		createGlass(
			world,
			glassSize,
			GRAVITY * 0.05f,
			0.5f);

		nph::Visualization* visualization = nph::Visualization::getInstance();
		if (visualization == nullptr)
		{
			return -1;
		}
		visualization->setCameraZoom(int(glassSize.x * 2.0f));
		visualization->setCameraPan(nph::Vec2(0.0f, glassSize.y * 0.5f));

		nph::WorldDrawSettings drawSettings;
		while (visualization->isRunning())
		{
			visualization->startFrame();
			visualization->drawWorld(world, drawSettings);
			visualization->endFrame();
		}
		return 0;
	}
	catch (const std::exception& exception)
	{
		nph::logError("Exception caught: ", exception.what());
	}
	catch (...)
	{
		nph::logError("Unknown exception caught.");
	}
	return -1;
}
