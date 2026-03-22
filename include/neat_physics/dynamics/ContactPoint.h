// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025-2026 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

// Includes
#include <algorithm>
#include "neat_physics/core/Platform.h"
#include "neat_physics/collision/CollisionPoint.h"
#include "neat_physics/dynamics/SolverSettings.h"
#include "neat_physics/body/Body.h"

namespace nph
{

/// Represents a contact point between two bodies 
/// constraining their relative motion
template <uint16_t D>
class ContactPoint
{
public:
	/// Array of tangent vectors
	using Tangents = std::array<Vec<D>, D - 1>;

	/// Default Constructor (non-initializing)
	ContactPoint() noexcept = default;

	/// Constructor
	explicit ContactPoint(const CollisionPoint<D>& point) noexcept :
		mPoint(point),
		mNormalImpulse(0.0f),
		mTangentImpulse(0.0f),
		mPersistent(false)
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

	/// Returns if the contact is persistent
	[[nodiscard]] bool isPersistent() const noexcept
	{
		return mPersistent;
	}

	/// Prepares the contact point for velocity solving;
	void prepareToSolve(
		const Tangents& tangents,
		const Transform<D>& transformA,
		const Transform<D>& transformB,
		const MassProperties<D>& massPropertiesA,
		const MassProperties<D>& massPropertiesB,
		LinearAngularPair<D>& velocityA,
		LinearAngularPair<D>& velocityB) noexcept;

	/// Solves the contact velocities
	/// asserts that friction is in [0, 1]
	void solveVelocities(
		const Tangents& tangents,
		const float friction,
		float invMassA,
		float invMassB,
		LinearAngularPair<D>& velocityA,
		LinearAngularPair<D>& velocityB) noexcept;

	/// Solves the contact position (penetration)
	void solvePositions(
		const SolverSettings& settings,
		Transform<D>& transformA,
		Transform<D>& transformB,
		MassProperties<D>& massPropertiesA,
		MassProperties<D>& massPropertiesB) noexcept;
	
private:

	/// Applies an impulse at the contact point
	void applyImpulse(
		Body<D>& bodyA,
		Body<D>& bodyB,
		const Vec<D>& impulse) const noexcept;

	/// Gets the transformed contact data
	void getTransformedContact(
		const Transform<D>& transformA,
		const Transform<D>& transformB,
		Vec<D>& normal,
		Vec<D>& clippedPoint,
		float& penetration) const noexcept;

	/// Collision point
	CollisionPoint<D> mPoint;

	/// Vector from the body A center of mass to the contact point
	Vec<D> mOffsetA;

	/// Vector from the body B center of mass to the contact point
	Vec<D> mOffsetB;

	/// Effective mass in the normal direction
	float mNormalMass;

	/// Effective mass in the tangent direction
	/// \todo: generalize for 3D
	std::array<float, D - 1> mTangentMasses;

	/// Accumulated normal impulse
	float mNormalImpulse;

	/// Precomputed invInertia * (offset cross normal) for body A and B
	AxisAngle<D> mNormalInvInertiaA;
	AxisAngle<D> mNormalInvInertiaB;

	/// Accumulated tangent (friction) impulse
	/// \todo: generalize for 3D
	std::array<float, D - 1> mTangentImpulse;

	/// Precomputed invInertia * (offset cross tangent) for body A and B
	std::array<AxisAngle<D>, D - 1> mTangentInvInertiaA;
	std::array<AxisAngle<D>, D - 1> mTangentInvInertiaB;

	/// Persistence flag
	bool mPersistent{ false };
};


template <uint16_t D>
void ContactPoint<D>::prepareToSolve(
	const Tangents& tangents,
	const Transform<D>& transformA,
	const Transform<D>& transformB,
	const MassProperties<D>& massPropertiesA,
	const MassProperties<D>& massPropertiesB,
	LinearAngularPair<D>& velocityA,
	LinearAngularPair<D>& velocityB) noexcept
{
	mOffsetA = mPoint.position - transformA.position;
	mOffsetB = mPoint.position - transformB.position;

	// Precompute normal mass, tangent mass, and bias.
	const Vec<D>& normal = mPoint.normal;
	mNormalMass = getEffectiveMass(
		massPropertiesA,
		massPropertiesB,
		mOffsetA,
		mOffsetB,
		normal);

	Vec<D> warmStartingImpulse = mNormalImpulse * normal;
	{
		mTangentMasses[0] = getEffectiveMass(
			massPropertiesA,
			massPropertiesB,
			mOffsetA,
			mOffsetB,
			tangents[0]);
		warmStartingImpulse += mTangentImpulse[0] * tangents[0];
	}

	if constexpr (D == 3)
	{
		mTangentMasses[1] = getEffectiveMass(
			massPropertiesA,
			massPropertiesB,
			mOffsetA,
			mOffsetB,
			tangents[1]);
		warmStartingImpulse += mTangentImpulse[1] * tangents[1];
	}

	// Precompute invInertia * (offset cross normal) for body A and B
	mNormalInvInertiaA = massPropertiesA.invWorldInertia * cross(mOffsetA, normal);
	mNormalInvInertiaB = massPropertiesB.invWorldInertia * cross(mOffsetB, normal);

	// Precompute invInertia * (offset cross tangent) for body A and B
	{
		mTangentInvInertiaA[0] = massPropertiesA.invWorldInertia * cross(mOffsetA, tangents[0]);
		mTangentInvInertiaB[0] = massPropertiesB.invWorldInertia * cross(mOffsetB, tangents[0]);
	}

	if constexpr (D == 3)
	{
		mTangentInvInertiaA[1] = massPropertiesA.invWorldInertia * cross(mOffsetA, tangents[1]);
		mTangentInvInertiaB[1] = massPropertiesB.invWorldInertia * cross(mOffsetB, tangents[1]);
	}

	// Apply the warm starting impulse
	velocityA.linear += massPropertiesA.invMass * -warmStartingImpulse;

	velocityA.angular +=
		massPropertiesA.invWorldInertia * cross(mOffsetA, -warmStartingImpulse);

	velocityB.linear += massPropertiesB.invMass * warmStartingImpulse;

	velocityB.angular +=
		massPropertiesB.invWorldInertia * cross(mOffsetB, warmStartingImpulse);
}

template <uint16_t D>
void ContactPoint<D>::solveVelocities(
	const Tangents& tangents,
	const float friction,
	float invMassA,
	float invMassB,
	LinearAngularPair<D>& velocityA,
	LinearAngularPair<D>& velocityB) noexcept
{
	assert(0.0f <= friction && friction <= 1.0f);

	const Vec<D>& normal = mPoint.normal;
	// Normal impulse
	{
		const float impulse = -mNormalMass * dot(
			normal,
			velocityB.linear + cross(velocityB.angular, mOffsetB) -
			velocityA.linear - cross(velocityA.angular, mOffsetA));

		const float oldImpulse = mNormalImpulse;
		mNormalImpulse = std::max(0.0f, oldImpulse + impulse);
		const float impulseChange = mNormalImpulse - oldImpulse;
		const Vec<D> impulseVec = impulseChange * normal;

		velocityA.linear -= invMassA * impulseVec;
		velocityA.angular -= impulseChange * mNormalInvInertiaA;

		velocityB.linear += invMassB * impulseVec;
		velocityB.angular += impulseChange * mNormalInvInertiaB;
	}

	// Dry friction impulse
	const float maxFriction = friction * mNormalImpulse;
	{
		const float impulse = -mTangentMasses[0] *
			dot(tangents[0],
				velocityB.linear + cross(velocityB.angular, mOffsetB) -
				velocityA.linear - cross(velocityA.angular, mOffsetA));

		const float oldImpulse = mTangentImpulse[0];
		mTangentImpulse[0] = std::clamp(
			oldImpulse + impulse,
			-maxFriction,
			maxFriction);

		const float impulseChange = mTangentImpulse[0] - oldImpulse;
		const Vec<D> impulseVec = impulseChange * tangents[0];

		velocityA.linear -= invMassA * impulseVec;
		velocityA.angular -= impulseChange * mTangentInvInertiaA[0];
		
		velocityB.linear += invMassB * impulseVec;
		velocityB.angular += impulseChange * mTangentInvInertiaB[0];
	}

	if constexpr (D == 3)
	{
		const float impulse = -mTangentMasses[1] *
			dot(tangents[1],
				velocityB.linear + cross(velocityB.angular, mOffsetB) -
				velocityA.linear - cross(velocityA.angular, mOffsetA));

		const float oldImpulse = mTangentImpulse[1];
		mTangentImpulse[1] = std::clamp(
			oldImpulse + impulse,
			-maxFriction,
			maxFriction);

		const float impulseChange = mTangentImpulse[1] - oldImpulse;
		const Vec<D> impulseVec = impulseChange * tangents[1];

		velocityA.linear -= invMassA * impulseVec;
		velocityA.angular -= impulseChange * mTangentInvInertiaA[1];

		velocityB.linear += invMassB * impulseVec;
		velocityB.angular += impulseChange * mTangentInvInertiaB[1];
	}
}

template <uint16_t D>
void ContactPoint<D>::solvePositions(
	const SolverSettings& settings,
	Transform<D>& transformA,
	Transform<D>& transformB,
	MassProperties<D>& massPropertiesA,
	MassProperties<D>& massPropertiesB) noexcept
{
	// This method is similar to the position based dynamics (PBD) approach :
	// we directly modify the positions and rotations of the bodies
	Vec<D> normal;
	float penetration;
	Vec<D> planePoint;
	getTransformedContact(transformA, transformB, normal, planePoint, penetration);

	const float biasFactor = std::max(
		0.0f,
		settings.getPositionCorrectionFactor() *
		(penetration - settings.getAllowedPenetration()));

	const Vec<D> offsetA = planePoint - transformA.position;
	const Vec<D> offsetB = planePoint - transformB.position;

	const float effectiveMass =
		getEffectiveMass(massPropertiesA, massPropertiesB, offsetA, offsetB, normal);

	const Vec<D> penetrationImpulse = std::max(0.0f, effectiveMass * biasFactor) * normal;

	// Directly integrate positions and rotations of the bodies in contact
	/// \todo generalize for 3D
	transformA.position -= massPropertiesA.invMass * penetrationImpulse;
	transformB.position += massPropertiesB.invMass * penetrationImpulse;

	if constexpr (D == 2)
	{
		transformA.rotation.set(transformA.rotation.get() -
			massPropertiesA.invWorldInertia * cross(offsetA, penetrationImpulse));

		transformB.rotation.set(transformB.rotation.get() +
			massPropertiesB.invWorldInertia * cross(offsetB, penetrationImpulse));
	}
	else
	{
		const Vec3 angVel = -(massPropertiesA.invWorldInertia * cross(offsetA, penetrationImpulse));
		const Quat angVelQuat(angVel.x, angVel.y, angVel.z, 0.0f);
		const Quat newRotationA = transformA.rotation.get() + 0.5f * (angVelQuat * transformA.rotation.get());
		transformA.rotation.set(newRotationA.getNormalized());
		massPropertiesA.updateWorldInertia(transformA.rotation);

		const Vec3 angVelB = massPropertiesB.invWorldInertia * cross(offsetB, penetrationImpulse);
		const Quat angVelQuatB(angVelB.x, angVelB.y, angVelB.z, 0.0f);
		const Quat newRotationB = transformB.rotation.get() + 0.5f * (angVelQuatB * transformB.rotation.get());
		transformB.rotation.set(newRotationB.getNormalized());
		massPropertiesB.updateWorldInertia(transformB.rotation);
	}
}

template <uint16_t D>
void ContactPoint<D>::getTransformedContact(
	const Transform<D>& transformA,
	const Transform<D>& transformB,
	Vec<D>& normal,
	Vec<D>& clippedPoint,
	float& penetration) const noexcept
{
	const std::array<Vec<D>, 2> positions{
		transformA.position,
		transformB.position };

	const std::array<Mat<D, D>, 2> rotations{
		transformA.rotation.getMat(),
		transformB.rotation.getMat() };

	const CollisionPoint<D>& contact = mPoint;
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
NPH_FORCE_INLINE [[nodiscard]] float getEffectiveMass(
	const MassProperties<D>& massPropertiesA,
	const MassProperties<D>& massPropertiesB,
	const Vec<D>& armA,
	const Vec<D>& armB,
	const Vec<D>& direction) noexcept
{
	const auto crossA = cross(armA, direction);
	const auto crossB = cross(armB, direction);
	const float invResult =
		massPropertiesA.invMass +
		massPropertiesB.invMass +
		dot(massPropertiesA.invWorldInertia * crossA, crossA) +
		dot(massPropertiesB.invWorldInertia * crossB, crossB);
	return 1.0f / invResult;
}

template <uint16_t D>
NPH_FORCE_INLINE void ContactPoint<D>::updateFrom(const ContactPoint<D>& other) noexcept
{
	mNormalImpulse = other.mNormalImpulse;
	mTangentImpulse = other.mTangentImpulse;
	mPersistent = true;
}

} // namespace nph
