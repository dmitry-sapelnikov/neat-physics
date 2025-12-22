// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

// Includes
#include "DrawFunctions.h"
#include "neat_physics/math/Mat33.h"

namespace nph
{

void drawBody(const Body<2>& body)
{
	const Mat22& rot = body.rotation.getMat();
	const Vec2& pos = body.position;
	const Vec2& hs = body.halfSize;

	const Vec2 v1 = pos + rot * Vec2(-hs.x, -hs.y);
	const Vec2 v2 = pos + rot * Vec2(hs.x, -hs.y);
	const Vec2 v3 = pos + rot * Vec2(hs.x, hs.y);
	const Vec2 v4 = pos + rot * Vec2(-hs.x, hs.y);

	glColor4f(1.0f, 1.0f, 0.9f, body.isStatic() ? 0.3f : 0.15f);
	glBegin(GL_TRIANGLE_FAN);
	glVertex3f(v1.x, v1.y, 0.0f);
	glVertex3f(v2.x, v2.y, 0.0f);
	glVertex3f(v3.x, v3.y, 0.0f);
	glVertex3f(v4.x, v4.y, 0.0f);
	glEnd();

	glColor3f(0.8f, 0.8f, 0.75f);
	glBegin(GL_LINE_LOOP);
	glVertex3f(v1.x, v1.y, 0.0f);
	glVertex3f(v2.x, v2.y, 0.0f);
	glVertex3f(v3.x, v3.y, 0.0f);
	glVertex3f(v4.x, v4.y, 0.0f);
	glEnd();
}

void drawBody(const Body<3>& body)
{
	const Mat33& rot = body.rotation.getMat();
	const Vec3& pos = body.position;
	const Vec3& hs = body.halfSize;
	const Vec3 v1 = pos + rot * Vec3(-hs.x, -hs.y, -hs.z);
	const Vec3 v2 = pos + rot * Vec3( hs.x, -hs.y, -hs.z);
	const Vec3 v3 = pos + rot * Vec3( hs.x,  hs.y, -hs.z);
	const Vec3 v4 = pos + rot * Vec3(-hs.x,  hs.y, -hs.z);
	const Vec3 v5 = pos + rot * Vec3(-hs.x, -hs.y,  hs.z);
	const Vec3 v6 = pos + rot * Vec3( hs.x, -hs.y,  hs.z);
	const Vec3 v7 = pos + rot * Vec3( hs.x,  hs.y,  hs.z);
	const Vec3 v8 = pos + rot * Vec3(-hs.x,  hs.y,  hs.z);

	// Draw box faces
	glColor4f(1.0f, 1.0f, 0.9f, body.isStatic() ? 0.3f : 0.15f);
	
	// Front face
	glBegin(GL_QUADS);

	setGlVertex(v1);
	setGlVertex(v2);
	setGlVertex(v3);
	setGlVertex(v4);

	// Back face
	setGlVertex(v5);
	setGlVertex(v6);
	setGlVertex(v7);
	setGlVertex(v8);

	// Left face
	setGlVertex(v1);
	setGlVertex(v4);
	setGlVertex(v8);
	setGlVertex(v5);

	// Right face
	setGlVertex(v2);
	setGlVertex(v6);
	setGlVertex(v7);
	setGlVertex(v3);

	// Top face
	setGlVertex(v4);
	setGlVertex(v3);
	setGlVertex(v7);
	setGlVertex(v8);

	// Bottom face
	setGlVertex(v1);
	setGlVertex(v2);
	setGlVertex(v6);
	setGlVertex(v5);

	glEnd();

	// Draw box edges
	glColor3f(0.8f, 0.8f, 0.75f);

	// Bottom edges
	glBegin(GL_LINE_LOOP);
	setGlVertex(v1);
	setGlVertex(v2);
	setGlVertex(v3);
	setGlVertex(v4);
	glEnd();

	// Top edges
	glBegin(GL_LINE_LOOP);
	setGlVertex(v5);
	setGlVertex(v6);
	setGlVertex(v7);
	setGlVertex(v8);
	glEnd();

	// Connect bottom and top edges
	glBegin(GL_LINES);
	setGlVertex(v1);
	setGlVertex(v5);
	setGlVertex(v2);
	setGlVertex(v6);
	setGlVertex(v3);
	setGlVertex(v7);
	setGlVertex(v4);
	setGlVertex(v8);
	glEnd();
}

void drawAabb(const Aabb<2>& aabb)
{
	glColor3f(0.0f, 0.5f, 0.0f);

	glBegin(GL_LINE_LOOP);
	glVertex2f(aabb.min.x, aabb.min.y);
	glVertex2f(aabb.max.x, aabb.min.y);
	glVertex2f(aabb.max.x, aabb.max.y);
	glVertex2f(aabb.min.x, aabb.max.y);
	glEnd();
}

void drawAabb(const Aabb<3>& aabb)
{
	glColor3f(0.0f, 0.5f, 0.0f);

	// Bottom face
	glBegin(GL_LINE_LOOP);
	glVertex3f(aabb.min.x, aabb.min.y, aabb.min.z);
	glVertex3f(aabb.max.x, aabb.min.y, aabb.min.z);
	glVertex3f(aabb.max.x, aabb.max.y, aabb.min.z);
	glVertex3f(aabb.min.x, aabb.max.y, aabb.min.z);
	glEnd();

	// Top face
	glBegin(GL_LINE_LOOP);
	glVertex3f(aabb.min.x, aabb.min.y, aabb.max.z);
	glVertex3f(aabb.max.x, aabb.min.y, aabb.max.z);
	glVertex3f(aabb.max.x, aabb.max.y, aabb.max.z);
	glVertex3f(aabb.min.x, aabb.max.y, aabb.max.z);
	glEnd();

	// Connect bottom and top faces
	glBegin(GL_LINES);
	glVertex3f(aabb.min.x, aabb.min.y, aabb.min.z);
	glVertex3f(aabb.min.x, aabb.min.y, aabb.max.z);
	glVertex3f(aabb.max.x, aabb.min.y, aabb.min.z);
	glVertex3f(aabb.max.x, aabb.min.y, aabb.max.z);
	glVertex3f(aabb.max.x, aabb.max.y, aabb.min.z);
	glVertex3f(aabb.max.x, aabb.max.y, aabb.max.z);
	glVertex3f(aabb.min.x, aabb.max.y, aabb.min.z);
	glVertex3f(aabb.min.x, aabb.max.y, aabb.max.z);
	glEnd();
}

} // namespace nph
