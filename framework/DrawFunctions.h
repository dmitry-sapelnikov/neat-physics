// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025-2026 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

// Includes
#include <random>
#include "glad/glad.h"
#include "neat_physics/World.h"
#include "neat_physics/math/Vec2.h"
#include "neat_physics/math/Vec3.h"
#include "Color.h"

namespace nph
{

/// Sets vertex for OpenGL based on dimension
inline void setGlVertex(const Vec2& v)
{
	glVertex2f(v.x, v.y);
}

/// Sets vertex for OpenGL based on dimension
inline void setGlVertex(const Vec3& v)
{
	glVertex3f(v.x, v.y, v.z);
}

/// Draws an arrow in 2D
template <uint16_t D>
void drawArrow(
	const Vec<D>& start,
	const Vec<D>& end,
	float tipSize,
	const Color& color)
{
	/// The ratio between the tip height and side length
	static constexpr float TIP_SIDE_FACTOR = 0.3f;
	assert(tipSize > 0.0f);

	const Vec<D> dir = end - start;
	const Vec<D> dirNorm = dir.getNormalized();

	Vec<D> orthoLeft;
	if constexpr (D == 2)
	{
		orthoLeft = cross(dirNorm, 1.0f);
	}
	else
	{
		orthoLeft = getNormalizedPerpendicular(dirNorm);
	}

	const Vec<D> tipEnd = end + tipSize * dirNorm;
	const Vec<D> leftArrowHead = end + TIP_SIDE_FACTOR * tipSize * orthoLeft;
	const Vec<D> rightArrowHead = end - TIP_SIDE_FACTOR * tipSize * orthoLeft;

	glColor3f(color.r, color.g, color.b);
	glBegin(GL_LINES);
	// Line
	setGlVertex(start);
	setGlVertex(end);
	// Arrowhead
	setGlVertex(leftArrowHead);
	setGlVertex(rightArrowHead);
	setGlVertex(tipEnd);
	setGlVertex(leftArrowHead);
	setGlVertex(tipEnd);
	setGlVertex(rightArrowHead);
	glEnd();
}

/// Draws a frame
template <uint16_t D>
void drawFrame(const Transform<D>& transform, float size)
{
	for (uint16_t i = 0; i < D; ++i)
	{
		Vec<D> axisEnd{};
		axisEnd[i] = size;
		axisEnd = transform * axisEnd;

		Color color;
		color[i] = 1.0f;
		drawArrow(
			transform.position,
			axisEnd,
			size * 0.2f,
			color);
	}
}

/// Draw contact points
template <uint16_t D>
void drawContactPoints(const World<D>& world, float pointSize)
{
	assert(pointSize > 0.0f);

	// Disable z-buffering to make sure contact points are visible
	glDisable(GL_DEPTH_TEST);
	glPointSize(pointSize);

	const ecs::Component<Transform<D>> transforms =
		world.getBodyStorage().getComponent<ecs::TransformTag>();

	for (const ContactManifold<D>& manifold : world.getContactSolver().getManifolds())
	{
		const Transform<D>& transformA = transforms[manifold.getBodyA()];
		const Transform<D>& transformB = transforms[manifold.getBodyB()];

		// Draw contact points
		for (uint32_t i = 0; i < manifold.getContactCount(); ++i)
		{
			const ContactPoint<D>& contact = manifold.getContact(i);
			const CollisionPoint<D>& point = contact.getPoint();
			// Mark persistent contacts in green, new contacts in red
			if (contact.isPersistent())
			{
				glColor3f(0.0f, 1.0f, 0.0f);
			}
			else
			{
				glColor3f(1.0f, 0.0f, 0.0f);
			}

			glBegin(GL_POINTS);
			setGlVertex(
				transformA.position +
				transformA.rotation.getMat() * point.localPoints[0]);

			setGlVertex(
				transformB.position +
				transformB.rotation.getMat() * point.localPoints[1]);
			glEnd();

		}
	}
	glEnd();
	glEnable(GL_DEPTH_TEST);
}

/// Draws contact areas
template <uint16_t D>
void drawContactAreas(const World<D>& world)
{
	// Create RNG to randomize contact colors
	std::mt19937 rng(0);
	std::uniform_real_distribution<float> colorDist(0.5f, 1.0f);

	// Disable z-buffering to make sure contact areas are visible
	glDisable(GL_DEPTH_TEST);
	if constexpr (D == 2)
	{
		glLineWidth(3.0f);
	}

	for (const auto& manifold : world.getContactSolver().getManifolds())
	{
		// Draw contact plate polygon
		glColor4f(
			colorDist(rng),
			colorDist(rng),
			colorDist(rng),
			0.5f);

		if constexpr (D == 2)
		{
			glBegin(GL_LINE_STRIP);
		}
		else
		{
			glBegin(GL_TRIANGLE_FAN);
		}
		setGlVertex(manifold.getContact(0).getPoint().position);
		for (uint32_t i = 1; i < manifold.getContactCount(); ++i)
		{
			setGlVertex(manifold.getContact(i).getPoint().position);
		}
		glEnd();
	}
	glEnable(GL_DEPTH_TEST);
}

/// Draw contact frames
template <uint16_t D>
void drawContactFrames(const World<D>& world, float axesSize)
{
	// Size of the arrow tip
	constexpr float ARROW_TIP_SIZE = 0.05f;

	assert(axesSize > 0.0f);

	// Disable z-buffering to make sure contact points are visible
	glDisable(GL_DEPTH_TEST);
	glLineWidth(1.0f);

	for (const auto& manifold : world.getContactSolver().getManifolds())
	{
		// Draw contact points
		for (uint32_t i = 0; i < manifold.getContactCount(); ++i)
		{
			const ContactPoint<D>& contact = manifold.getContact(i);
			const CollisionPoint<D>& point = contact.getPoint();

			// Draw contact normal arrow
			drawArrow(
				point.position,
				point.position + axesSize * manifold.getNormal(),
				ARROW_TIP_SIZE,
				Color(1.0f, 0.0f, 0.0f));

			// Draw contact tangent arrows
			for (uint16_t ti = 0; ti < D - 1; ++ti)
			{
				const Vec<D> tangent = manifold.getTangents()[ti];
				drawArrow(
					point.position,
					point.position + axesSize * tangent,
					ARROW_TIP_SIZE,
					Color(0.0f, ti == 0, ti != 0));
			}
		}
	}
	glEnd();
	// Re-enable z-buffering
	glEnable(GL_DEPTH_TEST);
}

/// Draws a 2D body
template <uint16_t D>
void drawBody(
	const Transform<D>& transform,
	const Vec<D>& halfSize,
	bool isStatic);

/// Draws an 2D Aabb
void drawAabb(const Aabb<2>& aabb, const Color& color);

/// Draws a 3D Aabb
void drawAabb(const Aabb<3>& aabb, const Color& color);

} // namespace nph
