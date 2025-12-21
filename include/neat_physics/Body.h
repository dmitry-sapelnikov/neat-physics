// A minimalistic 2D physics engine (https://github.com/dmitry-sapelnikov/neat-physics)
// SPDX-FileCopyrightText: 2025 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

// Includes
#include <vector>
#include "neat_physics/math/AxisAngle.h"
#include "neat_physics/math/Rotation.h"
#include "neat_physics/math/Inertia.h"

namespace nph
{

/// Box-shaped rigid body
/// The class design is intentionaly minimalistic
/// To achieve this, we use struct with public members
/// while keeping all members with value constraints constant
template <uint16_t D>
struct Body
{
	/// Half size (width / 2, height / 2)
	const Vec<D> halfSize;

	/// Mass (0 if static)
	const float mass;

	/// Inverse mass (0 if static)
	const float invMass;

	/// Moment of inertia (0 if static)
	const Inertia<D> inertia;

	/// Inverse moment of inertia (0 if static)
	const Inertia<D> invInertia;

	/// Friction coefficient [0, 1]
	const float friction;

	/// Position
	Vec<D> position{};

	/// Rotation
	Rotation<D> rotation{};

	/// Linear velocity
	Vec<D> linearVelocity{};

	/// Angular velocity
	AxisAngle<D> angularVelocity{};

	/// Constructor
	/// \param inSize Body size; must be > 0 in both dimensions
	/// \param inMass Body mass; if 0, the body is static; must be >= 0
	/// \param inFriction Friction coefficient; must be in range [0, 1]
	Body(
		const Vec2& inSize,
		float inMass,
		float inFriction) :

		halfSize(0.5f * inSize),
		mass(inMass),
		invMass((mass == 0.0f) ? 0.0f : 1.0f / mass),

		inertia(getBoxInertia(inSize, mass)),
		invInertia(getInvInertia(inertia)),

		friction(inFriction)
	{
		for (uint16_t i = 0; i < D; ++i)
		{
			assert(halfSize[i] > 0.0f);
		}
		assert(mass >= 0.0f);
		assert(0.0f <= friction && friction <= 1.0f);
	}

	/// Checks if the body is static
	[[nodiscard]] bool isStatic() const noexcept
	{
		return mass == 0.0f;
	}
};

/// 2D body alias
using Body2 = Body<2>;

/// 2D body array type
template <uint16_t D>
using BodyArray = std::vector<Body<D>>;

// namespace nph
}
