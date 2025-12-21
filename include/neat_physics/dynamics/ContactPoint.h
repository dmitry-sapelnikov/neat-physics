// A minimalistic 2D physics engine (https://github.com/dmitry-sapelnikov/neat-physics)
// SPDX-FileCopyrightText: 2025 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

// Includes
#include "neat_physics/collision/CollisionPoint.h"
#include "neat_physics/Body.h"

namespace nph
{

/// Represents a contact point between two bodies 
/// constraining their relative motion
class ContactPoint
{
public:
	/// Default Constructor (non-initializing)
	ContactPoint() noexcept = default;

	/// Constructor
	ContactPoint(const CollisionPoint& inPoint) noexcept :
		mPoint(inPoint),
		mNormalImpulse(0.0f),
		mTangentImpulse(0.0f)
		// The rest of members will be initialized in prepareToSolve
	{
	}

	/// Returns the collision point
	[[nodiscard]] const CollisionPoint& getPoint() const noexcept
	{
		return mPoint;
	}

	/// Updates the contact impulses from another one (for warm starting)
	void updateFrom(const ContactPoint& other) noexcept;

	/// Prepares the contact point for velocity solving;
	void prepareToSolve(
		Body<2>& bodyA,
		Body<2>& bodyB) noexcept;
	
	/// Solves the contact velocities
	/// asserts that friction is in [0, 1]
	void solveVelocities(
		Body<2>& bodyA,
		Body<2>& bodyB,
		float friction) noexcept;

	/// Solves the contact position (penetration)
	void solvePositions(Body<2>& bodyA, Body<2>& bodyB) noexcept;
	
private:
	/// Returns the relative velocity at the contact point
	[[nodiscard]] Vec2 getVelocityAtContact(
		const Body<2>& bodyA,
		const Body<2>& bodyB) const noexcept;

	/// Applies an impulse at the contact point
	void applyImpulse(
		Body<2>& bodyA,
		Body<2>& bodyB,
		const Vec2& impulse) const noexcept;

	/// Gets the transformed contact data
	void getTransformedContact(
		const Body<2>& bodyA,
		const Body<2>& bodyB,
		Vec2& normal,
		Vec2& clippedPoint,
		float& penetration) const;

	/// Collision point
	CollisionPoint mPoint;

	/// Tangent vector
	Vec2 mTangent;

	/// Vector from the body A center of mass to the contact point
	Vec2 mOffsetA;

	/// Vector from the body B center of mass to the contact point
	Vec2 mOffsetB;

	/// Effective mass in the normal direction
	float mNormalMass;

	/// Effective mass in the tangent direction
	float mTangentMass;

	/// Accumulated normal impulse
	float mNormalImpulse;

	/// Accumulated tangent (friction) impulse
	float mTangentImpulse;
};

}
