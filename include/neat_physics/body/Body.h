// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025-2026 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

// Includes
#include "neat_physics/World.h"

namespace nph
{

template <uint16_t D>
class World;

/// Box-shaped rigid body
/// The class design is intentionaly minimalistic
/// To achieve this, we use struct with public members
/// while keeping all members with value constraints constant
template <uint16_t D>
class Body
{
public:
	/// Constructor
	Body(World<D>& world, uint32_t id) noexcept :
		mWorld(&world),
		mId(id)
	{
	}

	/// Returns the body id, the index in the body array
	[[nodiscard]] uint32_t getId() const noexcept
	{
		return mId;
	}

	/// Returns the position
	[[nodiscard]] const Vec<D>& getPosition() const noexcept;

	/// Sets the position
	void setPosition(const Vec<D>& position) noexcept;

	/// Returns the rotation
	[[nodiscard]] const Rotation<D>& getRotation() const noexcept;

	/// Sets the rotation
	void setRotation(const typename Rotation<D>::RotationType& rotation) noexcept;

	/// Returns the transform (const version)
	[[nodiscard]] const Transform<D>& getTransform() const noexcept;

	/// Returns the linear velocity
	[[nodiscard]] const Vec<D>& getLinearVelocity() const noexcept;

	/// Sets the linear velocity
	void setLinearVelocity(const Vec<D>& linearVelocity) noexcept;

	/// Returns the angular velocity
	[[nodiscard]] const AxisAngle<D>& getAngularVelocity() const noexcept;

	/// Sets the angular velocity
	void setAngularVelocity(const AxisAngle<D>& angularVelocity) noexcept;

	/// Returns the linear + angular velocity
	[[nodiscard]] const LinearAngularPair<D>& getVelocity() const noexcept;

	/// Returns the inverse mass (0 if static)
	[[nodiscard]] float getInvMass() const noexcept;

	/// Returns the inverse moment of inertia in the world frame (0 if static)
	[[nodiscard]] const Inertia<D>& getInvWorldInertia() const noexcept;

	/// Returns the mass properties
	[[nodiscard]] const MassProperties<D>& getMassProperties() const noexcept;

	/// Checks if the body is static
	[[nodiscard]] bool isStatic() const noexcept;

	/// Returns the half size (width / 2, height / 2)
	[[nodiscard]] const Vec<D>& getHalfSize() const noexcept;

	/// Returns the friction coefficient [0, 1]
	[[nodiscard]] float getFriction() const noexcept;

	/// Sets the friction coefficient
	/// \param friction Friction coefficient; must be in range [0, 1]
	void setFriction(float friction) noexcept;

private:
	/// Returns the transform (non-const version)
	[[nodiscard]] Transform<D>& getTransform() noexcept;
	
	/// Returns the velocity pair (non-const version)
	[[nodiscard]] LinearAngularPair<D>& getVelocity() noexcept;

	/// Returns the mass properties (non-const version)
	[[nodiscard]] MassProperties<D>& getMassProperties() noexcept;

	/// Body id, the index in the body array
	uint32_t mId;

	/// World
	World<D>* mWorld;
};

/// 2D body array type
template <uint16_t D>
using BodyArray = std::vector<Body<D>>;

} // namespace nph

#include "Body.inl"
