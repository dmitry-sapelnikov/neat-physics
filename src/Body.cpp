// A minimalistic 2D physics engine (https://github.com/dmitry-sapelnikov/neat-physics)
// SPDX-FileCopyrightText: 2025 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#include "neat_physics/Body.h"

namespace nph
{

namespace
{

/// Returns the moment of inertia for a box shape
float getBoxInertia(const Vec2& size, float mass)
{
	return mass * size.lengthSquared() / 12.0f;
}

} // anonymous namespace

Body::Body(
	const Vec2& inSize,
	float inMass,
	float inFriction) :

	halfSize(0.5f * inSize),

	mass(inMass),
	invMass((mass == 0.0f) ? 0.0f : 1.0f / mass),

	inertia(getBoxInertia(inSize, mass)),
	invInertia((mass == 0.0f) ? 0.0f : 1.0f / inertia),

	friction(inFriction)
{
	assert(halfSize.x > 0.0f);
	assert(halfSize.y > 0.0f);
	assert(mass >= 0.0f);
	assert(0.0f <= friction && friction <= 1.0f);
}

// namespace nph
}
