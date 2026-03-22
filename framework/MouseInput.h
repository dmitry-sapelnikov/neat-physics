#pragma once

// Includes
#include "neat_physics/math/Vec2.h"

namespace nph
{

/// Mouse input state
struct MouseInput
{
	/// Mouse position
	Vec2 position{ 0.0f, 0.0f };

	/// Left mouse down flag
	bool leftMouseDown = false;

	/// Left mouse clicked flag
	bool leftMouseClicked = false;

	/// Middle mouse down flag
	bool middleMouseDown = false;

	/// Right mouse down flag
	bool rightMouseDown = false;

	/// Mouse wheel delta
	double mouseWheel = 0.0;
};

} // namespace nph
