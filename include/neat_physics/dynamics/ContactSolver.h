// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025-2026 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

// Includes
#include <unordered_map>
#include "neat_physics/collision/CollisionPairMap.h"
#include "neat_physics/collision/CollisionCallback.h"
#include "neat_physics/dynamics/ContactManifold.h"
#include "neat_physics/dynamics/SolverSettings.h"

namespace nph
{

/// Solver for contact constraints between bodies
template <uint16_t D>
class ContactSolver : public CollisionCallback<D>
{
public:
	/// Array of contact manifolds
	using ManifoldsArray = std::vector<ContactManifold<D>>;

	/// Body view layout
	using BodyLayout = ecs::Layout<
		ecs::TransformBlock<D>,
		ecs::VelocityBlock<D>,
		ecs::MassPropertiesBlock<D>,
		ecs::FrictionBlock>;

	using Bodies = ecs::MutableView<BodyLayout>;

	/// Constructor
	/// \note all parameters must reference to persistent objects
	ContactSolver(
		const Bodies& bodies,
		CollisionPairMap& collisionPairMap,
		const SolverSettings& settings) noexcept :

		mBodies(bodies),
		mCollisionPairMap(collisionPairMap),
		mSettings(settings)
	{
	}

	/// Clears all contact manifolds
	void clear() noexcept;

	/// Returns the contact manifolds
	[[nodiscard]] const ManifoldsArray& getManifolds() const noexcept
	{
		return mManifolds;
	}

	/// Prepares the contact manifolds update
	void prepareManifoldsUpdate() noexcept;

	/// Collision callback
	void onCollision(
		const CollisionManifold<D>& collisionManifold,
		uint32_t& broadPhasePairTag) override;

	/// Finishes the contact manifolds update
	void finishManifoldsUpdate();

	/// Prepares the contact solver for velocity solving
	void prepareToSolve() noexcept;

	/// Solves the contact velocities
	void solveVelocities() noexcept;

	/// Solves the contact positions (penetration)
	void solvePositions() noexcept;

private:
	/// Body view for accessing body components
	Bodies mBodies;

	/// Reference to the collision pairs map
	CollisionPairMap& mCollisionPairMap;

	/// Reference to the solver settings
	const SolverSettings& mSettings;

	/// Contact manifolds
	ManifoldsArray mManifolds;
};

template <uint16_t D>
void ContactSolver<D>::clear() noexcept
{
	mManifolds.clear();
}

template <uint16_t D>
void ContactSolver<D>::prepareToSolve() noexcept
{
	const ecs::Component<Transform<D>> transforms =
		mBodies.getComponent<ecs::TransformTag>();

	const ecs::Component<MassProperties<D>> massProperties =
		mBodies.getComponent<ecs::MassPropertiesTag>();

	ecs::Component<LinearAngularPair<D>> velocities =
		mBodies.getComponent<ecs::VelocityTag>();

	for (auto& manifold : mManifolds)
	{
		manifold.prepareToSolve(
			transforms,
			massProperties,
			velocities);
	}
}

template <uint16_t D>
void ContactSolver<D>::solveVelocities() noexcept
{
	const ecs::Component<MassProperties<D>> massProperties =
		mBodies.getComponent<ecs::MassPropertiesTag>();

	ecs::Component<LinearAngularPair<D>> velocities =
		mBodies.getComponent<ecs::VelocityTag>();

	for (uint32_t i = 0; i < mSettings.getVelocityIterations(); ++i)
	{
		for (auto& manifold : mManifolds)
		{
			manifold.solveVelocities(
				massProperties,
				velocities);
		}
	}
}

template <uint16_t D>
void ContactSolver<D>::solvePositions() noexcept
{
	ecs::Component<Transform<D>> transforms =
		mBodies.getComponent<ecs::TransformTag>();

	ecs::Component<MassProperties<D>> massProperties =
		mBodies.getComponent<ecs::MassPropertiesTag>();

	for (uint32_t i = 0; i < mSettings.getPositionIterations(); ++i)
	{
		for (auto& manifold : mManifolds)
		{
			manifold.solvePositions(
				mSettings,
				transforms,
				massProperties);
		}
	}
}

template <uint16_t D>
void ContactSolver<D>::prepareManifoldsUpdate() noexcept
{
	for (auto& pair : mManifolds)
	{
		pair.markObsolete();
	}
}

template <uint16_t D>
void ContactSolver<D>::onCollision(
	const CollisionManifold<D>& manifold,
	uint32_t& broadPhasePairTag)
{
	if (broadPhasePairTag != CollisionPairMap::NULL_VALUE)
	{
		mManifolds[broadPhasePairTag].update(manifold);
	}
	else
	{
		size_t manifoldCount = mManifolds.size();
		assert(manifoldCount <= std::numeric_limits<uint32_t>::max());
		const uint32_t index = static_cast<uint32_t>(manifoldCount);
		broadPhasePairTag = index;

		ecs::Component<float> frictions = mBodies.getComponent<ecs::FrictionTag>();
		mManifolds.emplace_back(
			manifold,
			frictions[manifold.bodyA],
			frictions[manifold.bodyB]);
	}
}

template <uint16_t D>
void ContactSolver<D>::finishManifoldsUpdate()
{
	// Remove obsolete manifolds, rewire the contact pairs map
	uint32_t mi = 0;
	while (mi != mManifolds.size())
	{
		auto& manifold = mManifolds[mi];
		if (manifold.isObsolete())
		{
			// Mark as removed
			uint32_t* obsoletePairTag = mCollisionPairMap.at(
				manifold.getBodyA(),
				manifold.getBodyB());

			if (obsoletePairTag != nullptr)
			{
				*obsoletePairTag = CollisionPairMap::NULL_VALUE;
			}

			// Swap and pop
			if (mi != mManifolds.size() - 1)
			{
				manifold = std::move(mManifolds.back());

				uint32_t* pairTag = mCollisionPairMap.at(
					manifold.getBodyA(),
					manifold.getBodyB());

				if (pairTag != nullptr)
				{
					*pairTag = mi;
				}
			}
			mManifolds.pop_back();
		}
		else
		{
			++mi;
		}
	}
}

} // nph namespace
