// A minimalistic 2D physics engine (https://github.com/dmitry-sapelnikov/neat-physics)
// SPDX-FileCopyrightText: 2025 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

// Includes
#include "neat_physics/dynamics/ContactPoint.h"
#include <algorithm>

namespace nph
{

namespace
{

/// Applies impulse at a point relative to the center of mass
void applyImpulse(
	Body<2>& body,
	const Vec2& localPoint,
	const Vec2& impulse) noexcept
{
	body.linearVelocity += body.invMass * impulse;
	body.angularVelocity += body.invInertia * cross(localPoint, impulse);
}

/// Computes the effective mass for a given contact and direction
[[nodiscard]] float getEffectiveMass(
	const Body<2>& bodyA,
	const Body<2>& bodyB,
	const Vec2& armA,
	const Vec2& armB,
	const Vec2& direction) noexcept
{
	const float crossA = cross(armA, direction);
	const float crossB = cross(armB, direction);
	const float invResult =
		bodyA.invMass + bodyB.invMass +
		bodyA.invInertia * crossA * crossA +
		bodyB.invInertia * crossB * crossB;
	return 1.0f / invResult;
}

} // anonymous namespace

void ContactPoint::updateFrom(const ContactPoint& other) noexcept
{
	mNormalImpulse = other.mNormalImpulse;
	mTangentImpulse = other.mTangentImpulse;
}

void ContactPoint::prepareToSolve(
	Body<2>& bodyA,
	Body<2>& bodyB) noexcept
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

void ContactPoint::solveVelocities(
	Body<2>& bodyA,
	Body<2>& bodyB,
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

void ContactPoint::solvePositions(
	Body<2>& bodyA,
	Body<2>& bodyB) noexcept
{
	// This method is similar to the position based dynamics (PBD) approach :
	// we directly modify the positions and rotations of the bodies

	// Position correction factor
	static constexpr float POSITION_CORRECTION_FACTOR = 0.2f;

	// Allowed penetration between geometries
	static constexpr float ALLOWED_PENETRATION = 0.001f;

	Vec2 normal;
	float penetration;
	Vec2 planePoint;
	getTransformedContact(bodyA, bodyB, normal, planePoint, penetration);

	const float biasFactor = std::max(
		0.0f,
		POSITION_CORRECTION_FACTOR * (penetration - ALLOWED_PENETRATION));

	const Vec2 offsetA = planePoint - bodyA.position;
	const Vec2 offsetB = planePoint - bodyB.position;

	const float effectiveMass =
		getEffectiveMass(bodyA, bodyB, offsetA, offsetB, normal);

	const Vec2 penetrationImpulse = std::max(0.0f, effectiveMass * biasFactor) * normal;

	// Directly integrate positions and rotations of the bodies in contact
	bodyA.position -= bodyA.invMass * penetrationImpulse;
	bodyA.rotation.setAngle(bodyA.rotation.getAngle() -
		bodyA.invInertia * cross(offsetA, penetrationImpulse));

	bodyB.position += bodyB.invMass * penetrationImpulse;
	bodyB.rotation.setAngle(bodyB.rotation.getAngle() +
		bodyB.invInertia * cross(offsetB, penetrationImpulse));
}

/// Returns the relative velocity at the contact point
[[nodiscard]] Vec2 ContactPoint::getVelocityAtContact(
	const Body<2>& bodyA,
	const Body<2>& bodyB) const noexcept
{
	return
		bodyB.linearVelocity + cross(bodyB.angularVelocity, mOffsetB) -
		bodyA.linearVelocity - cross(bodyA.angularVelocity, mOffsetA);
}

/// Applies an impulse at the contact point
void ContactPoint::applyImpulse(
	Body<2>& bodyA,
	Body<2>& bodyB,
	const Vec2& impulse) const noexcept
{
	nph::applyImpulse(bodyA, mOffsetA, -impulse);
	nph::applyImpulse(bodyB, mOffsetB,  impulse);
}

void ContactPoint::getTransformedContact(
	const Body<2>& bodyA,
	const Body<2>& bodyB,
	Vec2& normal,
	Vec2& clippedPoint,
	float& penetration) const
{
	const std::array<Vec2, 2> positions{
		bodyA.position,
		bodyB.position };

	const std::array<Mat22, 2> rotations{
		bodyA.rotation.getMat(),
		bodyB.rotation.getMat() };

	const CollisionPoint<2>& contact = mPoint;
	const uint32_t ind1 = contact.clipBoxIndex;
	const uint32_t ind2 = 1 - ind1;

	clippedPoint =
		positions[ind2] +
		rotations[ind2] * contact.localPoints[ind2];

	normal = rotations[ind1] * contact.localContactNormal;

	const Vec2 planePoint =
		positions[ind1] +
		rotations[ind1] * contact.localPoints[ind1];

	penetration = dot(planePoint - clippedPoint, normal);

	// Normal must point from A to B
	normal = (ind1 == 0) ? normal : -normal;
}

} // namespace nph
