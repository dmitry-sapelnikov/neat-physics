// Includes
#include "neat_physics/dynamics/ContactPoint.h"
#include <algorithm>

namespace nph
{

namespace
{

/// Applies impulse at a point relative to the center of mass
static void applyImpulse(
	Body& body,
	const Vec2& localPoint,
	const Vec2& impulse)
{
	body.linearVelocity += body.invMass * impulse;
	body.angularVelocity += body.invInertia * cross(localPoint, impulse);
}

} // anonymous namespace

void ContactPoint::updateFrom(const ContactPoint& other) noexcept
{
	mNormalImpulse = other.mNormalImpulse;
	mTangentImpulse = other.mTangentImpulse;
}

void ContactPoint::prepareToSolve(
	Body& bodyA,
	Body& bodyB,
	float invTimeStep)
{
	// Baumgarte stabilization factor
	static constexpr float BAUMGARTE_STAB = 0.2f;

	// Allowed penetration between geometries
	static constexpr float ALLOWED_PENETRATION = 0.01f;

	assert(invTimeStep > 0.0f);

	mOffsetA = mPoint.position - bodyA.position;
	mOffsetB = mPoint.position - bodyB.position;

	// Precompute normal mass, tangent mass, and bias.
	mNormalMass = getEffectiveMass(bodyA, bodyB, mPoint.normal);

	mTangent = cross(mPoint.normal, 1.0f);
	mTangentMass = getEffectiveMass(bodyA, bodyB, mTangent);

	mNormalBias = BAUMGARTE_STAB * invTimeStep *
		std::max(0.0f, mPoint.penetration - ALLOWED_PENETRATION);

	// Apply the warm starting impulse
	applyImpulse(
		bodyA,
		bodyB,
		mNormalImpulse * mPoint.normal + mTangentImpulse * mTangent);
}

void ContactPoint::solveVelocities(
	Body& bodyA,
	Body& bodyB,
	float friction)
{
	assert(0.0f <= friction && friction <= 1.0f);

	// Normal impulse
	{
		float impulse = mNormalMass * (
			-dot(getVelocityAtContact(bodyA, bodyB), mPoint.normal) +
			mNormalBias);

		float oldAccImpulse = mNormalImpulse;
		mNormalImpulse = std::max(0.0f, oldAccImpulse + impulse);
		applyImpulse(
			bodyA,
			bodyB,
			(mNormalImpulse - oldAccImpulse) * mPoint.normal);
	}

	// Dry friction impulse
	{
		Vec2 tangent = cross(mPoint.normal, 1.0f);
		float maxFriction = friction * mNormalImpulse;

		float impulse = (-mTangentMass) *
			dot(getVelocityAtContact(bodyA, bodyB), tangent);

		float oldAccImpulse = mTangentImpulse;
		mTangentImpulse = std::clamp(
			oldAccImpulse + impulse,
			-maxFriction,
			maxFriction);

		applyImpulse(
			bodyA,
			bodyB,
			(mTangentImpulse - oldAccImpulse) * tangent);
	}
}

[[nodiscard]] Vec2 ContactPoint::getVelocityAtContact(
	const Body& bodyA,
	const Body& bodyB) const noexcept
{
	return
		bodyB.linearVelocity + cross(bodyB.angularVelocity, mOffsetB) -
		bodyA.linearVelocity - cross(bodyA.angularVelocity, mOffsetA);
}

[[nodiscard]] float ContactPoint::getEffectiveMass(
	const Body& bodyA,
	const Body& bodyB,
	const Vec2& direction) const noexcept
{
	const float dotA = dot(mOffsetA, direction);
	const float dotB = dot(mOffsetB, direction);
	const float invResult =
		(bodyA.invMass + bodyB.invMass) +
		(bodyA.invInertia * (mOffsetA.lengthSquared() - dotA * dotA) +
		 bodyB.invInertia * (mOffsetB.lengthSquared() - dotB * dotB));
	return 1.0f / invResult;
}

void ContactPoint::applyImpulse(
	Body& bodyA,
	Body& bodyB,
	const Vec2& impulse) const noexcept
{
	nph::applyImpulse(bodyA, mOffsetA, -impulse);
	nph::applyImpulse(bodyB, mOffsetB,  impulse);
}

} // namespace nph
