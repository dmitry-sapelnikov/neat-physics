// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

// Includes
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
		orthoLeft = cross(dirNorm, Vec3(1.0f, 0.0f, 0.0f));
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
void drawFrame(const Vec<D>& position, const Mat<D, D>& rotation, float size)
{
	for (uint16_t i = 0; i < D; ++i)
	{
		Vec<D> axisEnd{};
		axisEnd[i] = size;
		axisEnd = position + rotation * axisEnd;
		Color color;
		color[i] = 1.0f;
		drawArrow(
			position,
			axisEnd,
			size * 0.2f,
			color);
	}
}

/// Draw contact points
template <uint16_t D>
void drawContacts(const World<D>& world, float pointSize)
{
	assert(pointSize > 0.0f);
	glPointSize(pointSize);
	glColor3f(1.0f, 0.0f, 0.0f);
	glBegin(GL_POINTS);
	for (const auto& pair : world.getContactSolver().getManifolds())
	{
		const ContactManifold<D>& manifold = pair.second;
		const Body<D>& bodyA = manifold.getBodyA();
		const Body<D>& bodyB = manifold.getBodyB();

		for (uint32_t i = 0; i < manifold.getContactCount(); ++i)
		{
			const CollisionPoint<D>& point = manifold.getContact(i).getPoint();
			const Vec<D> point1 =
				bodyA.position + bodyA.rotation.getMat() * point.localPoints[0];

			const Vec<D> point2 =
				bodyB.position + bodyB.rotation.getMat() * point.localPoints[1];

			// Draw contact point on body A
			setGlVertex(point1);

			// Draw contact point on body B
			setGlVertex(point2);
		}
	}
	glEnd();
}

/// Draws a 2D body
void drawBody(const Body<2>& body);

/// Draws a 3D body
void drawBody(const Body<3>& body);

/// Draws an 2D Aabb
void drawAabb(const Aabb<2>& aabb);

/// Draws a 3D Aabb
void drawAabb(const Aabb<3>& aabb);

} // namespace nph
