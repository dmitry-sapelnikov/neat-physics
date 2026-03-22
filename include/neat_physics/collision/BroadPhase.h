// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025-2026 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

// Includes
#include <algorithm>
#include <neat_physics/body/Body.h>
#include <neat_physics/collision/AabbTree.h>
#include <neat_physics/collision/BoxGeometry.h>
#include <neat_physics/collision/CollisionPairMap.h>
#include <neat_physics/collision/BroadPhaseCallback.h>
#include <neat_physics/collision/CollisionSettings.h>

namespace nph
{

/// Broad-phase collision detection using sweep-and-prune algorithm
template <uint16_t D>
class BroadPhase
{
public:
	/// Body view layout
	using BodyLayout = ecs::Layout<
		ecs::TransformBlock<D>,
		ecs::MassPropertiesBlock<D>,
		ecs::HalfSizeBlock<D>>;

	/// Body view alias
	using Bodies = ecs::ConstantView<BodyLayout>;

	/// Array of body AABBs
	using AabbArray = std::vector<Aabb<D>>;

	/// Constructor
	BroadPhase(
		const Bodies& bodies,
		CollisionPairMap& collisionPairs,
		const CollisionSettings& settings) noexcept :

		mBodies(bodies),
		mAabbTree(),
		mCollisionPairs(collisionPairs),
		mSettings(settings)
	{
	}

	/// Clears the broad-phase data
	void clear() noexcept
	{
		mAabbTree.clear();
		mAabbTreeNodeIds.clear();
		mAabbs.clear();
		mBodyMoveFlags.clear();
		mCollisionPairs.clear();
	}

	/// Returns the AABBs for all bodies
	[[nodiscard]] const AabbArray& getAabbs() const noexcept
	{
		return mAabbs;
	}

	/// Returns the AABB tree
	[[nodiscard]] const AabbTree<D>& getAabbTree() const noexcept
	{
		return mAabbTree;
	}

	/// Updates the pairs of bodies which AABBs are overlapping
	void update(BroadPhaseCallback& callback)
	{
		mBodyMoveFlags.assign(mBodies.size(), false);
		mMovedBodies.clear();

		auto transforms = mBodies.getComponent<ecs::TransformTag>();
		auto halfSizes = mBodies.getComponent<ecs::HalfSizeTag>();
		auto massProperties = mBodies.getComponent<ecs::MassPropertiesTag>();

		for (uint32_t i = 0; i < transforms.size(); ++i)
		{
			const Transform<D>& bodyTransform = transforms[i];
			const Vec<D>& bodyHalfSize = halfSizes[i];

			// We need to expand the AABB by the separation factor
			// to make broad phase and narrow phase consistent
			const Aabb<D> aabb = getAabb(
				bodyTransform.position,
				bodyTransform.rotation.getMat(),
				bodyHalfSize).getExpanded(mSettings.getSeparationFactor());

			if (i < mAabbs.size())
			{
				mAabbs[i] = aabb;
				if (mAabbTree.moveAabb(
					mAabbTreeNodeIds[i],
					aabb,
					mSettings.getAabbTreeExpansionFactor()))
				{
					mBodyMoveFlags[i] = true;
					mMovedBodies.push_back(i);
				}
			}
			else
			{
				mAabbs.push_back(aabb);
				mBodyMoveFlags[i] = true;
				mMovedBodies.push_back(i);
				const uint32_t nodeId = mAabbTree.addAabb(
					aabb,
					mSettings.getAabbTreeExpansionFactor(),
					i);
				mAabbTreeNodeIds.push_back(nodeId);
			}
		}

		// Prune collision pairs
		if (!mMovedBodies.empty())
		{
			mCollisionPairs.startUpdate(mBodyMoveFlags);

			// Insert new collision pairs
			for (uint32_t bodyInd : mMovedBodies)
			{
				const uint32_t nodeId = mAabbTreeNodeIds[bodyInd];
				mAabbTree.testOverlap(
					mAabbTree.getAabb(nodeId),
					[this, &massProperties, nodeId, bodyInd](uint32_t curNodeId)
					{
						if (curNodeId == nodeId)
						{
							return;
						}

						const uint32_t otherBodyInd = mAabbTree.getUserData(curNodeId);
						uint32_t indA = bodyInd;
						uint32_t indB = otherBodyInd;

						if ((massProperties[indA].isStatic() && massProperties[indB].isStatic()) ||
							(mBodyMoveFlags[otherBodyInd] && indA > indB))
						{
							return;
						}

						if (indA > indB)
						{
							std::swap(indA, indB);
						}

						mCollisionPairs.addPair(indA, indB);
					});
			}
		}

		mCollisionPairs.endUpdate(
			[this, &callback](uint32_t bodyA, uint32_t bodyB, uint32_t& broadPhasePairTag)
			{
				if (mAabbs[bodyA].overlaps(mAabbs[bodyB]))
				{
					callback.onCollision(bodyA, bodyB, broadPhasePairTag);
				}
				else
				{
					broadPhasePairTag = CollisionPairMap::NULL_VALUE;
				}
			});
	}

private:
	/// Reference to the body array
	Bodies mBodies;

	/// Reference to the collision pairs map
	CollisionPairMap& mCollisionPairs;

	/// Reference to the collision settings
	const CollisionSettings& mSettings;

	/// AABB tree for broad-phase collision detection
	AabbTree<D> mAabbTree;

	/// AABB tree node IDs for each body index
	std::vector<uint32_t> mAabbTreeNodeIds;

	/// AABBs for each body index
	/// The AABBs consider the separation factor in settings
	AabbArray mAabbs;

	/// Flags indicating whether a body has moved during the current step
	CollisionPairMap::BodyMoveFlags mBodyMoveFlags;

	/// List of body indices which AABBs were moved during the current step
	std::vector<uint32_t> mMovedBodies;
};

} // namespace nph
