// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025-2026 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

// Includes
#include "neat_physics/core/Ecs.h"
#include "neat_physics/collision/CollisionManifold.h"
#include "neat_physics/dynamics/ContactPoint.h"

namespace nph
{

/// Persistent contact manifold between two bodies
/// Exploits temporal coherence to improve precision
template <uint16_t D>
class ContactManifold
{
public:
	/// Array of tangent vectors
	using Tangents = std::array<Vec<D>, D - 1>;

	/// Transform components
	using TransformComponent = ecs::Component<Transform<D>>;

	/// Mass properties components
	using MassPropertiesComponent = ecs::Component<MassProperties<D>>;

	/// Velocity component
	using VelocityComponent = ecs::Component<LinearAngularPair<D>>;

	/// Constructor
	/// \todo: handle friction updates
	ContactManifold(
		const CollisionManifold<D>& manifold,
		float frictionA,
		float frictionB) noexcept;

	/// Returns the first body
	uint32_t getBodyA() const noexcept
	{
		return mBodyA;
	}

	/// Returns the second body
	uint32_t getBodyB() const noexcept
	{
		return mBodyB;
	}

	/// Returns the contact friction coefficient
	float getFriction() const noexcept
	{
		return mFriction;
	}

	/// Returns the normal vector, pointing from body A to body B
	const Vec<D>& getNormal() const noexcept
	{
		return mNormal;
	}

	/// Returns the tangent vectors
	const Tangents& getTangents() const noexcept
	{
		return mTangents;
	}

	/// Returns the contact count
	inline [[nodiscard]] uint32_t getContactCount() const noexcept
	{
		return mContactCount;
	}

	/// Returns the contact at given index
	inline const ContactPoint<D>& getContact(uint32_t index) const noexcept
	{
		assert(index < mContactCount);
		return mContacts[index];
	}

	/// Returns if the manifold is obsolete
	[[nodiscard]] bool isObsolete() const noexcept
	{
		return mObsolete;
	}

	/// Marks the manifold as obsolete
	void markObsolete() noexcept
	{
		mObsolete = true;
	}

	/// Updates the contact manifold with new contacts
	/// preserving impulses for matching contact points
	void update(const CollisionManifold<D>& newManifold) noexcept;

	/// Prepares the contact manifold for velocity solving
	void prepareToSolve(
		const TransformComponent& transforms,
		const MassPropertiesComponent& massProperties,
		VelocityComponent& velocities) noexcept;

	/// Solves the contact velocities
	void solveVelocities(
		const MassPropertiesComponent& massProperties,
		VelocityComponent& velocities) noexcept;

	/// Solves the contact positions (penetration)
	void solvePositions(
		const SolverSettings& settings,
		TransformComponent& transforms,
		MassPropertiesComponent& massProperties) noexcept;

private:
	/// Updates the normal and tangent vectors from the collision manifold
	void updateNormalAndTangents(
		const CollisionManifold<D>& manifold) noexcept;
	
	/// Body A
	uint32_t mBodyA;

	/// Body B
	uint32_t mBodyB;

	/// Normal vector, pointing from body A to body B
	Vec<D> mNormal;

	/// Tangent vectors
	std::array<Vec<D>, D - 1> mTangents;

	/// Contact array
	std::array<ContactPoint<D>, CollisionPoint<D>::MAX_POINTS> mContacts;

	/// Actual contact count
	uint32_t mContactCount;

	/// Obsoletion flag
	bool mObsolete;

	/// Contact pair friction coefficient
	float mFriction;
};

template <uint16_t D>
ContactManifold<D>::ContactManifold(
	const CollisionManifold<D>& manifold,
	float frictionA,
	float frictionB) noexcept :

	mBodyA(manifold.bodyA),
	mBodyB(manifold.bodyB),
	mContacts(),
	mContactCount(manifold.pointsCount),
	mObsolete(false),

	// A well-known approximation for friction between two materials
	// \todo: introduce material pairs
	mFriction(std::sqrt(frictionA * frictionB))
{
	assert(manifold.bodyA < manifold.bodyB);
	assert(frictionA >= 0.0f);
	assert(frictionB >= 0.0f);
	assert(mContactCount > 0);
	updateNormalAndTangents(manifold);

	for (uint32_t i = 0; i < mContactCount; ++i)
	{
		mContacts[i] = ContactPoint<D>(manifold.points[i]);
	}
	assert(0 < mContactCount && mContactCount <= CollisionPoint<D>::MAX_POINTS);
}

template <uint16_t D>
void ContactManifold<D>::update(
	const CollisionManifold<D>& newManifold) noexcept
{
	constexpr float PERSISTENCE_TOLERANCE = 0.01f;

	updateNormalAndTangents(newManifold);

	// Make a backup of old contacts
	std::array<ContactPoint<D>, CollisionPoint<D>::MAX_POINTS> oldContacts;
	const uint32_t oldCount = mContactCount;
	for (uint32_t i = 0; i < mContactCount; ++i)
	{
		oldContacts[i] = mContacts[i];
	}

	for (uint32_t i = 0; i < newManifold.pointsCount; ++i)
	{
		auto& contact = mContacts[i];
		contact = ContactPoint<D>(newManifold.points[i]);
		const auto& point = contact.getPoint();

		for (ContactPoint<D>*oldContact = oldContacts.data();
			oldContact < oldContacts.data() + oldCount;
			++oldContact)
		{
			const auto& oldPoint = oldContact->getPoint();
			if (point.featurePair == oldPoint.featurePair ||
				(point.localPoints[0] - oldPoint.localPoints[0]).lengthSquared() < 
					PERSISTENCE_TOLERANCE * PERSISTENCE_TOLERANCE ||
				(point.localPoints[1] - oldPoint.localPoints[1]).lengthSquared() <
					PERSISTENCE_TOLERANCE * PERSISTENCE_TOLERANCE)
			{
				contact.updateFrom(*oldContact);
				break;
			}
		}
	}
	mContactCount = newManifold.pointsCount;
	mObsolete = false;
}

template <uint16_t D>
void ContactManifold<D>::prepareToSolve(
	const TransformComponent& transforms,
	const MassPropertiesComponent& massProperties,
	VelocityComponent& velocities) noexcept
{
	for (ContactPoint<D>*contact = mContacts.data();
		contact < mContacts.data() + mContactCount;
		++contact)
	{
		contact->prepareToSolve(
			mTangents,
			transforms[mBodyA],
			transforms[mBodyB],
			massProperties[mBodyA],
			massProperties[mBodyB],
			velocities[mBodyA],
			velocities[mBodyB]);
	}
}

template <uint16_t D>
void ContactManifold<D>::solveVelocities(
	const MassPropertiesComponent& massProperties,
	VelocityComponent& velocities) noexcept
{
	float invMassA = massProperties[mBodyA].invMass;
	float invMassB = massProperties[mBodyB].invMass;

	LinearAngularPair<D>& velocityA = velocities[mBodyA];
	LinearAngularPair<D>& velocityB = velocities[mBodyB];

	for (ContactPoint<D>*contact = mContacts.data();
		contact < mContacts.data() + mContactCount;
		++contact)
	{
		contact->solveVelocities(
			mTangents,
			mFriction,
			invMassA,
			invMassB,
			velocityA,
			velocityB);
	}
}

template <uint16_t D>
void ContactManifold<D>::solvePositions(
	const SolverSettings& settings,
	TransformComponent& transforms,
	MassPropertiesComponent& massProperties) noexcept
{
	MassProperties<D>& massPropertiesA = massProperties[mBodyA];
	MassProperties<D>& massPropertiesB = massProperties[mBodyB];

	Transform<D>& transformA = transforms[mBodyA];
	Transform<D>& transformB = transforms[mBodyB];

	for (ContactPoint<D>*contact = mContacts.data();
		contact < mContacts.data() + mContactCount;
		++contact)
	{
		contact->solvePositions(
			settings,
			transformA,
			transformB,
			massPropertiesA,
			massPropertiesB);
	}
}

template <uint16_t D>
void ContactManifold<D>::updateNormalAndTangents(
	const CollisionManifold<D>& manifold) noexcept
{
	/// \todo: remove this implicit assumption 
	/// that all contact points in the manifold have the same normal
	mNormal = manifold.points[0].normal;

	if constexpr (D == 2)
	{
		mTangents[0] = cross(mNormal, 1.0f);
	}
	else
	{
		mTangents[0] = getNormalizedPerpendicular(mNormal);
		mTangents[1] = cross(mNormal, mTangents[0]);
	}
}

} // namespace nph
