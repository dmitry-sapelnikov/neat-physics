// A minimalistic 2D physics engine (https://github.com/dmitry-sapelnikov/neat-physics)
// SPDX-FileCopyrightText: 2025 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

// Includes
#include "neat_physics/collision/BroadPhase.h"
#include <algorithm>
#include <iterator>

namespace nph
{

void BroadPhase::update(BroadPhaseCallback& callback)
{
	// Reserve-emplace because Aabb is immutable
	mAabbs.clear();
	mAabbs.reserve(mBodies.size());
	for (size_t i = 0; i < mBodies.size(); ++i)
	{
		mAabbs.emplace_back(getAabb(
			mBodies[i].position,
			mBodies[i].rotation.getMat(),
			mBodies[i].halfSize));
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
	for (int32_t ei = static_cast<int32_t>(mEndpoints.size() >> 1);
		ei < static_cast<int32_t>(mBodies.size()); ++ei) // Endpoint index
	{
		mEndpoints.emplace_back(0.0f, ei, true);
		mEndpoints.emplace_back(0.0f, ei, false);
	}

	// Update endpoint positions
	for (auto& endpoint : mEndpoints)
	{
		endpoint.position = endpoint.isStart ?
			mAabbs[ endpoint.index ].min.x :
			mAabbs[ endpoint.index ].max.x;
	}

	// \todo Explicitly use the insertion sort
	std::sort(mEndpoints.begin(), mEndpoints.end());
	sweepAxis(callback);
}

void BroadPhase::sweepAxis(BroadPhaseCallback& callback)
{
	mActivePoints.clear();
	for (const auto& endpoint : mEndpoints)
	{
		if (endpoint.isStart)
		{
			const uint32_t i1 = endpoint.index;
			const Body<2>& bodyA = mBodies[i1];
			const Aabb2& aabbA = mAabbs[i1];

			for (uint32_t i2 : mActivePoints)
			{
				if (bodyA.isStatic() && mBodies[i2].isStatic())
				{
					continue;
				}

				// If y-axes don't intersect
				const Aabb2& aabbB = mAabbs[i2];
				if (aabbA.max.y < aabbB.min.y ||
					aabbB.max.y < aabbA.min.y)
				{
					continue;
				}

				if (i1 < i2)
				{
					callback.onCollision(i1, i2);
				}
				else
				{
					callback.onCollision(i2, i1);
				}
			}
			mActiveMapping[endpoint.index] = static_cast<uint32_t>(mActivePoints.size());
			mActivePoints.push_back(i1);
		}
		else
		{
			// Swap and pop
			const uint32_t idx = mActiveMapping[endpoint.index];
			const uint32_t last = mActivePoints.back();
			mActivePoints[idx] = last;
			mActiveMapping[last] = idx;
			mActivePoints.pop_back();
		}
	}
}

} // namespace nph
