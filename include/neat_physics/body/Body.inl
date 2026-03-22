// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025-2026 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

// Includes
#include "neat_physics/World.h"

namespace nph
{

/// Returns the position
template <uint16_t D>
NPH_FORCE_INLINE const Vec<D>& Body<D>::getPosition() const noexcept
{
	return getTransform().position;
}

/// Sets the position
template <uint16_t D>
NPH_FORCE_INLINE void Body<D>::setPosition(const Vec<D>& position) noexcept
{
	getTransform().position = position;
}

template <uint16_t D>
NPH_FORCE_INLINE const Rotation<D>& Body<D>::getRotation() const noexcept
{
	return getTransform().rotation;
}

template <uint16_t D>
NPH_FORCE_INLINE void Body<D>::setRotation(
	const typename Rotation<D>::RotationType& rotationValue) noexcept
{
	auto& rotation = getTransform().rotation;
	rotation.set(rotationValue);
	if constexpr (D == 3)
	{
		getMassProperties().updateWorldInertia(rotation);
	}
}

template <uint16_t D>
NPH_FORCE_INLINE const Transform<D>& Body<D>::getTransform() const noexcept
{
	return mWorld->getBodyStorage().get<ecs::TransformTag>(mId);
}

template <uint16_t D>
NPH_FORCE_INLINE Transform<D>& Body<D>::getTransform() noexcept
{
	return mWorld->getBodyStorage().get<ecs::TransformTag>(mId);
}

template <uint16_t D>
NPH_FORCE_INLINE const Vec<D>& Body<D>::getLinearVelocity() const noexcept
{
	return getVelocity().linear;
}

template <uint16_t D>
NPH_FORCE_INLINE void Body<D>::setLinearVelocity(
	const Vec<D>& linearVelocity) noexcept
{
	getVelocity().linear = linearVelocity;
}

template <uint16_t D>
NPH_FORCE_INLINE const AxisAngle<D>& Body<D>::getAngularVelocity() const noexcept
{
	return getVelocity().angular;
}

template <uint16_t D>
NPH_FORCE_INLINE void Body<D>::setAngularVelocity(
	const AxisAngle<D>& angularVelocity) noexcept
{
	getVelocity().angular = angularVelocity;
}

template <uint16_t D>
NPH_FORCE_INLINE const LinearAngularPair<D>& Body<D>::getVelocity() const noexcept
{
	return mWorld->getBodyStorage().get<ecs::VelocityTag>(mId);
}

template <uint16_t D>
NPH_FORCE_INLINE LinearAngularPair<D>& Body<D>::getVelocity() noexcept
{
	return mWorld->getBodyStorage().get<ecs::VelocityTag>(mId);
}

template <uint16_t D>
NPH_FORCE_INLINE float Body<D>::getInvMass() const noexcept
{
	return getMassProperties().invMass;
}

template <uint16_t D>
NPH_FORCE_INLINE const Inertia<D>& Body<D>::getInvWorldInertia() const noexcept
{
	return getMassProperties().invWorldInertia;
}

template <uint16_t D>
NPH_FORCE_INLINE const MassProperties<D>& Body<D>::getMassProperties() const noexcept
{
	return mWorld->getBodyStorage().get<ecs::MassPropertiesTag>(mId);
}

template <uint16_t D>
NPH_FORCE_INLINE MassProperties<D>& Body<D>::getMassProperties() noexcept
{
	return mWorld->getBodyStorage().get<ecs::MassPropertiesTag>(mId);
}

template <uint16_t D>
NPH_FORCE_INLINE bool Body<D>::isStatic() const noexcept
{
	return getInvMass() == 0.0f;
}

template <uint16_t D>
NPH_FORCE_INLINE const Vec<D>& Body<D>::getHalfSize() const noexcept
{
	return mWorld->getBodyStorage().get<ecs::HalfSizeTag>(mId);
}

template <uint16_t D>
NPH_FORCE_INLINE float Body<D>::getFriction() const noexcept
{
	return mWorld->getBodyStorage().get<ecs::FrictionTag>(mId);
}

template <uint16_t D>
NPH_FORCE_INLINE void Body<D>::setFriction(float friction) noexcept
{
	assert(0.0f <= friction && friction <= 1.0f);
	mWorld->getBodyStorage().get<ecs::FrictionTag>(mId) = friction;
}

} // namespace nph
