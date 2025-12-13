#pragma once

// Includes
#include <vector>
#include "neat_physics/math/Rotation.h"

namespace nph
{

/// Box-shaped rigid body
/// The class design is intentionaly minimalistic
/// To achieve this, we use struct with public members
/// while keeping all members with value constraints constant
struct Body
{
	/// Half size (width / 2, height / 2)
	const Vec2 halfSize;

	/// Mass (0 if static)
	const float mass;

	/// Inverse mass (0 if static)
	const float invMass;

	/// Moment of inertia (0 if static)
	const float inertia;

	/// Inverse moment of inertia (0 if static)
	const float invInertia;

	/// Friction coefficient [0, 1]
	const float friction;

	/// Position
	Vec2 position{ 0.0f, 0.0f };

	/// Rotation
	Rotation rotation{ 0.0f };

	/// Linear velocity
	Vec2 linearVelocity{ 0.0f, 0.0f };

	/// Angular velocity
	float angularVelocity{ 0.0f };

	/// Constructor
	/// \param inSize Body size; must be > 0 in both dimensions
	/// \param inMass Body mass; if 0, the body is static; must be >= 0
	/// \param inFriction Friction coefficient; must be in range [0, 1]
	Body(
		const Vec2& inSize,
		float inMass,
		float inFriction);

	/// Checks if the body is static
	[[nodiscard]] bool isStatic() const noexcept
	{
		return mass == 0.0f;
	}
};

/// Body array type
using BodyArray = std::vector<Body>;

// namespace nph
}
