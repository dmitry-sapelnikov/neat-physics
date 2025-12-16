// A minimalistic 2D physics engine (https://github.com/dmitry-sapelnikov/neat-physics)
// SPDX-FileCopyrightText: 2025 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

// Includes
#include "neat_physics/collision/BroadPhase.h"
#include <algorithm>
#include <iterator>

namespace nph
{

namespace
{

/// Computes the AABB for a box-shaped body
Aabb getAabb(const Body& body)
{
	const Mat22 absRotation = abs(body.rotation.getMat());
	const Vec2 extents =
		body.halfSize.x * absRotation.col1 +
		body.halfSize.y * absRotation.col2;

	return {
		body.position - extents,
		body.position + extents
	};
}

} // anonymous namespace

void BroadPhase::update()
{
	// Reserve-emplace because Aabb is immutable
	mAabbs.clear();
	mAabbs.reserve(mBodies.size());
	for (size_t i = 0; i < mBodies.size(); ++i)
	{
		mAabbs.emplace_back(getAabb(mBodies[i]));
	}

	mActiveMapping.resize(mBodies.size());
	// Very inefficient, but it is really actual since
	// currently there is no body removal,only clearing all
	if (mEndpoints.size() > mBodies.size() * 2)
	{
		mEndpoints.clear();
	}

	// Add endpoints for bodies that have been added since the last update
	assert(mEndpoints.size() % 2 == 0);
	for (int32_t i = static_cast<int32_t>(mEndpoints.size() >> 1);
		 i < static_cast<int32_t>(mBodies.size()); ++i)
	{
		mEndpoints.emplace_back(0.0f, i + 1);
		mEndpoints.emplace_back(0.0f, -(i + 1));
	}

	// Update endpoint positions
	for (auto& endpoint : mEndpoints)
	{
		endpoint.position = endpoint.index > 0 ?
			mAabbs[ endpoint.index - 1].min.x :
			mAabbs[ -endpoint.index - 1].max.x;
	}

	// \todo Explicitly use the insertion sort
	std::sort(mEndpoints.begin(), mEndpoints.end());
	sweepAxis();
}

void BroadPhase::sweepAxis()
{
	mCollidingPairs.clear();
	mActivePoints.clear();

	const Body* bodyStart = mBodies.data();
	const Aabb* aabbStart = mAabbs.data();
	for (const auto& endpoint : mEndpoints)
	{
		if (endpoint.index > 0)
		{
			const uint32_t i1 = static_cast<uint32_t>(endpoint.index - 1);
			const Body* bodyA = bodyStart + i1;
			const Aabb* aabbA = aabbStart + i1;

			for (uint32_t i2 : mActivePoints)
			{
				if (bodyA->isStatic() && (bodyStart + i2)->isStatic())
				{
					continue;
				}

				// If y-axes don't intersect
				const Aabb* aabbB = aabbStart + i2;
				if (aabbA->max.y < aabbB->min.y || 
					aabbB->max.y < aabbA->min.y)
				{
					continue;
				}

				if (i1 < i2)
				{
					mCollidingPairs.emplace_back(i1, i2);
				}
				else
				{
					mCollidingPairs.emplace_back(i2, i1);
				}
			}
			mActiveMapping[endpoint.index - 1] = static_cast<uint32_t>(mActivePoints.size());
			mActivePoints.push_back(i1);
		}
		else
		{
			// Swap and pop
			const uint32_t idx = mActiveMapping[-endpoint.index - 1];
			const uint32_t last = mActivePoints.back();
			mActivePoints[idx] = last;
			mActiveMapping[last] = idx;
			mActivePoints.pop_back();
		}
	}
}

} // namespace nph
