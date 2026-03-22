// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025-2026 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

#include <vector>
#include "neat_physics/body/BodyEcs.h"
#include "neat_physics/collision/CollisionSystem.h"
#include "neat_physics/dynamics/ContactSolver.h"

namespace nph
{

/// World settings
struct WorldSettings
{
	/// Collision settings
	CollisionSettings collisionSettings;

	/// Solver settings
	SolverSettings solverSettings;
};

/// Physics world
template <uint16_t D>
class World
{
public:
	using BodyStorage = ecs::Storage<ecs::Layout<
		ecs::TransformBlock<D>,
		ecs::VelocityBlock<D>,
		ecs::MassPropertiesBlock<D>,
		ecs::HalfSizeBlock<D>,
		ecs::FrictionBlock>>;

	/// Constructor
	/// \param gravity Gravity vector applied to all bodies
	/// \param settings World simulation settings
	World(
		const Vec<D>& gravity,
		const WorldSettings& settings = {});

	/// Reserves memory for bodies
	/// \note The number of bodies is intentionally limited to uint32_t
	void reserveBodies(uint32_t maxBodies);

	/// Returns the body storage in the world
	[[nodiscard]] const BodyStorage& getBodyStorage() const noexcept
	{
		return mBodyStorage;
	}

	/// Returns the collision system
	[[nodiscard]] const CollisionSystem<D>& getCollision() const noexcept
	{
		return mCollision;
	}

	/// Returns the contact solver
	[[nodiscard]] const ContactSolver<D>& getContactSolver() const noexcept
	{
		return mContactSolver;
	}

	/// Adds a body to the world
	/// \return the added body or nullptr if the body could not be added
	/// (e.g., when the number of bodies == uint32_t max value)
	Body<D> addBody(
		const Vec<D>& size,
		float mass,
		float friction,
		const Vec<D>& position = Vec<D>(0.0f),
		const AxisAngle<D>& axisAngleRad = AxisAngle<D>(0.0f));

	/// Clear the world: remove all bodies
	void clear() noexcept;

	/// Perform one simulation step
	void doStep(float dt);

	/// Returns the world settings (const version)
	[[nodiscard]] const WorldSettings& getSettings() const noexcept
	{
		return mSettings;
	}

	/// Returns the world settings (non-const version)
	[[nodiscard]] WorldSettings& getSettings() noexcept
	{
		return mSettings;
	}

private:
	/// Applies forces to all bodies
	void applyForces(float timeStep);

	/// Integrates positions of all bodies
	void integratePositions(float timeStep);

	/// Gravity vector
	Vec<D> mGravity;

	/// World settings
	WorldSettings mSettings;

	/// Body storage
	BodyStorage mBodyStorage;

	/// Map of collision pairs to contact manifold indices
	CollisionPairMap mCollisionPairs;

	/// Collision system
	CollisionSystem<D> mCollision;

	/// Contact solver
	ContactSolver<D> mContactSolver;
};

template <uint16_t D>
World<D>::World(
	const Vec<D>& gravity,
	const WorldSettings& settings) :

	mGravity(gravity),
	mSettings(settings),
	mCollision(
		mBodyStorage.createConstantView<CollisionSystem<D>::BodyLayout>(),
		mCollisionPairs,
		mSettings.collisionSettings),
	mContactSolver(
		mBodyStorage.createMutableView<ContactSolver<D>::BodyLayout>(),
		mCollisionPairs,
		mSettings.solverSettings)
{
}

template <uint16_t D>
void World<D>::reserveBodies(uint32_t maxBodies)
{
	mBodyStorage.reserve(maxBodies);
}

template <uint16_t D>
Body<D> World<D>::addBody(
	const Vec<D>& size,
	float mass,
	float friction,
	const Vec<D>& position,
	const AxisAngle<D>& axisAngleRad)
{
	size_t bodyId = mBodyStorage.create();
	assert(bodyId <= std::numeric_limits<uint32_t>::max());

	Transform<D>& transform = mBodyStorage.get<ecs::TransformTag>(bodyId);
	transform.position = position;

	LinearAngularPair<D>& velocity = mBodyStorage.get<ecs::VelocityTag>(bodyId);
	velocity.linear = Vec<D>(0.0f);
	velocity.angular = AxisAngle<D>(0.0f);

	MassProperties<D> massProperties = mBodyStorage.get<ecs::MassPropertiesTag>(bodyId);
	massProperties.invMass = (mass > 0.0f) ? 1.0f / mass : 0.0f;
	const Inertia<D> invInertia = getInvInertia(getBoxInertia(size, mass));
	massProperties.invLocalInertia = invInertia;
	massProperties.invWorldInertia = invInertia;
	mBodyStorage.get<ecs::MassPropertiesTag>(bodyId) = massProperties;

	mBodyStorage.get<ecs::HalfSizeTag>(bodyId) = 0.5f * size;
	mBodyStorage.get<ecs::FrictionTag>(bodyId) = friction;

	Body<D> result(*this, static_cast<uint32_t>(bodyId));
	result.setRotation(axisAngleRad);
	return result;
}

template <uint16_t D>
void World<D>::clear() noexcept
{
	mBodyStorage.clear();
	mCollision.clear();
	mContactSolver.clear();
}

template <uint16_t D>
void World<D>::doStep(float timeStep)
{
	assert(timeStep > 0.0f);
	applyForces(timeStep);

	mContactSolver.prepareManifoldsUpdate();
	mCollision.update(mContactSolver);
	mContactSolver.finishManifoldsUpdate();

	mContactSolver.prepareToSolve();
	mContactSolver.solveVelocities();
	integratePositions(timeStep);
	// Solving of positions is intetionally done after the integration step
	mContactSolver.solvePositions();
}

template <uint16_t D>
void World<D>::applyForces(float timeStep)
{
	auto massProperties = mBodyStorage.getComponent<ecs::MassPropertiesTag>();
	auto velocities = mBodyStorage.getComponent<ecs::VelocityTag>();

	for (size_t i = 0; i < velocities.size(); ++i)
	{
		velocities[i].linear +=
			(!massProperties[i].isStatic()) * timeStep * mGravity;
	}
}

template <uint16_t D>
void World<D>::integratePositions(float timeStep)
{
	auto transforms = mBodyStorage.getComponent<ecs::TransformTag>();
	auto velocities = mBodyStorage.getComponent<ecs::VelocityTag>();
	auto massProperties = mBodyStorage.getComponent<ecs::MassPropertiesTag>();

	for (size_t i = 0; i < transforms.size(); ++i)
	{
		auto& transform = transforms[i];
		auto& velocity = velocities[i];

		transform.position += timeStep * velocity.linear;
		if constexpr (D == 2)
		{
			transform.rotation.set(
				transform.rotation.get() + timeStep * velocity.angular);
		}
		else
		{
			const Vec3& angularVelocity = velocity.angular;
			const Quat velocityQuat(
				angularVelocity.x,
				angularVelocity.y,
				angularVelocity.z,
				0.f);

			const Quat angularDelta =
				(0.5f * timeStep) * (velocityQuat * transform.rotation.get());

			transform.rotation.set(
				(transform.rotation.get() + angularDelta).getNormalized());

			massProperties[i].updateWorldInertia(transform.rotation);
		}
	}
}

} // namespace nph
