// A minimalistic 2D physics engine (https://github.com/dmitry-sapelnikov/neat-physics)
// SPDX-FileCopyrightText: 2025 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

// Includes
#include <algorithm>
#include "neat_physics/collision/CollisionPoint.h"
#include "neat_physics/Body.h"

namespace nph
{

/// Represents a contact point between two bodies 
/// constraining their relative motion
template <uint16_t D>
class ContactPoint
{
public:
	/// Default Constructor (non-initializing)
	ContactPoint() noexcept = default;

	/// Constructor
	ContactPoint(const CollisionPoint<D>& inPoint) noexcept :
		mPoint(inPoint),
		mNormalImpulse(0.0f),
		mTangentImpulse(0.0f)
		// The rest of members will be initialized in prepareToSolve
	{
	}

	/// Returns the collision point
	[[nodiscard]] const CollisionPoint<D>& getPoint() const noexcept
	{
		return mPoint;
	}

	/// Updates the contact impulses from another one (for warm starting)
	void updateFrom(const ContactPoint& other) noexcept;

	/// Prepares the contact point for velocity solving;
	void prepareToSolve(
		Body<D>& bodyA,
		Body<D>& bodyB) noexcept;

	/// Solves the contact velocities
	/// asserts that friction is in [0, 1]
	void solveVelocities(
		Body<D>& bodyA,
		Body<D>& bodyB,
		float friction) noexcept;

	/// Solves the contact position (penetration)
	void solvePositions(Body<D>& bodyA, Body<D>& bodyB) noexcept;
	
private:
	/// Returns the relative velocity at the contact point
	[[nodiscard]] Vec<D> getVelocityAtContact(
		const Body<D>& bodyA,
		const Body<D>& bodyB) const noexcept;

	/// Applies an impulse at the contact point
	void applyImpulse(
		Body<D>& bodyA,
		Body<D>& bodyB,
		const Vec<D>& impulse) const noexcept;

	/// Gets the transformed contact data
	void getTransformedContact(
		const Body<D>& bodyA,
		const Body<D>& bodyB,
		Vec<D>& normal,
		Vec<D>& clippedPoint,
		float& penetration) const;

	/// Collision point
	CollisionPoint<D> mPoint;

	/// Tangent vector
	Vec<D> mTangent;

	/// Vector from the body A center of mass to the contact point
	Vec<D> mOffsetA;

	/// Vector from the body B center of mass to the contact point
	Vec<D> mOffsetB;

	/// Effective mass in the normal direction
	float mNormalMass;

	/// Effective mass in the tangent direction
	/// \todo: generalize for 3D
	float mTangentMass;

	/// Accumulated normal impulse
	float mNormalImpulse;

	/// Accumulated tangent (friction) impulse
	/// \todo: generalize for 3D
	float mTangentImpulse;
};

/// Applies impulse at a point relative to the center of mass
template <uint16_t D>
void applyImpulse(
	Body<D>& body,
	const Vec<D>& localPoint,
	const Vec<D>& impulse) noexcept
{
	body.linearVelocity += body.invMass * impulse;
	body.angularVelocity += body.invInertia * cross(localPoint, impulse);
}

template <uint16_t D>
void ContactPoint<D>::prepareToSolve(
	Body<D>& bodyA,
	Body<D>& bodyB) noexcept
{
	mOffsetA = mPoint.position - bodyA.position;
	mOffsetB = mPoint.position - bodyB.position;

	// Precompute normal mass, tangent mass, and bias.
	mNormalMass = getEffectiveMass(bodyA, bodyB, mOffsetA, mOffsetB, mPoint.normal);

	mTangent = cross(mPoint.normal, 1.0f);
	mTangentMass = getEffectiveMass(bodyA, bodyB, mOffsetA, mOffsetB, mTangent);

	// Apply the warm starting impulse
	applyImpulse(
		bodyA,
		bodyB,
		mNormalImpulse * mPoint.normal + mTangentImpulse * mTangent);
}

template <uint16_t D>
void ContactPoint<D>::solveVelocities(
	Body<D>& bodyA,
	Body<D>& bodyB,
	float friction) noexcept
{
	assert(0.0f <= friction && friction <= 1.0f);

	// Normal impulse
	{
		const float impulse = -mNormalMass *
			dot(getVelocityAtContact(bodyA, bodyB), mPoint.normal);

		const float oldImpulse = mNormalImpulse;
		mNormalImpulse = std::max(0.0f, oldImpulse + impulse);
		applyImpulse(
			bodyA,
			bodyB,
			(mNormalImpulse - oldImpulse) * mPoint.normal);
	}

	// Dry friction impulse
	{
		const float maxFriction = friction * mNormalImpulse;

		const float impulse = -mTangentMass *
			dot(getVelocityAtContact(bodyA, bodyB), mTangent);

		const float oldImpulse = mTangentImpulse;
		mTangentImpulse = std::clamp(
			oldImpulse + impulse,
			-maxFriction,
			maxFriction);

		applyImpulse(
			bodyA,
			bodyB,
			(mTangentImpulse - oldImpulse) * mTangent);
	}
}

template <uint16_t D>
void ContactPoint<D>::solvePositions(
	Body<D>& bodyA,
	Body<D>& bodyB) noexcept
{
	// This method is similar to the position based dynamics (PBD) approach :
	// we directly modify the positions and rotations of the bodies

	// Position correction factor
	static constexpr float POSITION_CORRECTION_FACTOR = 0.2f;

	// Allowed penetration between geometries
	static constexpr float ALLOWED_PENETRATION = 0.001f;

	Vec<D> normal;
	float penetration;
	Vec<D> planePoint;
	getTransformedContact(bodyA, bodyB, normal, planePoint, penetration);

	const float biasFactor = std::max(
		0.0f,
		POSITION_CORRECTION_FACTOR * (penetration - ALLOWED_PENETRATION));

	const Vec<D> offsetA = planePoint - bodyA.position;
	const Vec<D> offsetB = planePoint - bodyB.position;

	const float effectiveMass =
		getEffectiveMass(bodyA, bodyB, offsetA, offsetB, normal);

	const Vec<D> penetrationImpulse = std::max(0.0f, effectiveMass * biasFactor) * normal;

	// Directly integrate positions and rotations of the bodies in contact
	bodyA.position -= bodyA.invMass * penetrationImpulse;
	bodyA.rotation.setAngle(bodyA.rotation.getAngle() -
		bodyA.invInertia * cross(offsetA, penetrationImpulse));

	bodyB.position += bodyB.invMass * penetrationImpulse;
	bodyB.rotation.setAngle(bodyB.rotation.getAngle() +
		bodyB.invInertia * cross(offsetB, penetrationImpulse));
}

template <uint16_t D>
/// Returns the relative velocity at the contact point
[[nodiscard]] Vec<D> ContactPoint<D>::getVelocityAtContact(
	const Body<D>& bodyA,
	const Body<D>& bodyB) const noexcept
{
	return
		bodyB.linearVelocity + cross(bodyB.angularVelocity, mOffsetB) -
		bodyA.linearVelocity - cross(bodyA.angularVelocity, mOffsetA);
}

/// Applies an impulse at the contact point
template <uint16_t D>
void ContactPoint<D>::applyImpulse(
	Body<D>& bodyA,
	Body<D>& bodyB,
	const Vec<D>& impulse) const noexcept
{
	nph::applyImpulse(bodyA, mOffsetA, -impulse);
	nph::applyImpulse(bodyB, mOffsetB, impulse);
}

template <uint16_t D>
void ContactPoint<D>::getTransformedContact(
	const Body<D>& bodyA,
	const Body<D>& bodyB,
	Vec<D>& normal,
	Vec<D>& clippedPoint,
	float& penetration) const
{
	const std::array<Vec<D>, 2> positions{
		bodyA.position,
		bodyB.position };

	const std::array<Mat<D, D>, 2> rotations{
		bodyA.rotation.getMat(),
		bodyB.rotation.getMat() };

	const CollisionPoint<2>& contact = mPoint;
	const uint32_t ind1 = contact.clipBoxIndex;
	const uint32_t ind2 = 1 - ind1;

	clippedPoint =
		positions[ind2] +
		rotations[ind2] * contact.localPoints[ind2];

	normal = rotations[ind1] * contact.localContactNormal;

	const Vec<D> planePoint =
		positions[ind1] +
		rotations[ind1] * contact.localPoints[ind1];

	penetration = dot(planePoint - clippedPoint, normal);

	// Normal must point from A to B
	normal = (ind1 == 0) ? normal : -normal;
}

/// Computes the effective mass for a given contact and direction
/// \todo: generalize for 3D
template <uint16_t D>
[[nodiscard]] float getEffectiveMass(
	const Body<D>& bodyA,
	const Body<D>& bodyB,
	const Vec<D>& armA,
	const Vec<D>& armB,
	const Vec<D>& direction) noexcept
{
	const auto crossA = cross(armA, direction);
	const auto crossB = cross(armB, direction);
	const float invResult =
		bodyA.invMass +
		bodyB.invMass +
		dot(cross((bodyA.invInertia * crossA), armA) +
			cross((bodyB.invInertia * crossB), armB), direction);
	return 1.0f / invResult;
}

template <uint16_t D>
void ContactPoint<D>::updateFrom(const ContactPoint<D>& other) noexcept
{
	mNormalImpulse = other.mNormalImpulse;
	mTangentImpulse = other.mTangentImpulse;
}

} // namespace nph
