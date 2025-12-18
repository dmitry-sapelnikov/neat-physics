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
Aabb getAabb(const Body& body) noexcept
{
	const Mat22 absRotation = abs(body.rotation.getMat());
	const Vec2 halfExtents =
		body.halfSize.x * absRotation.col1 +
		body.halfSize.y * absRotation.col2;

	return {
		body.position - halfExtents,
		body.position + halfExtents
	};
}

} // anonymous namespace

void BroadPhase::prepareSweep()
{
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
			mAabbs[endpoint.index].min.x :
			mAabbs[endpoint.index].max.x;
	}

	// \todo Explicitly use the insertion sort
	std::sort(mEndpoints.begin(), mEndpoints.end());
}

} // namespace nph
