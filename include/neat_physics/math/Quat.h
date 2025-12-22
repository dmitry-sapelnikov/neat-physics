// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

//	Includes
#include "neat_physics/math/Mat33.h"

namespace nph
{
/// Quat in convention (x*i, y*j, z*k, w)
struct Quat
{
	/// X component
	float x;

	/// Y component
	float y;

	/// Z component
	float z;

	/// W component
	float w;

	/// Default constructor (no initialization)
	Quat() noexcept = default;

	///	All-components constructor
	constexpr Quat(
		float inX,
		float inY,
		float inZ,
		float inW) noexcept :

		x(inX),
		y(inY),
		z(inZ),
		w(inW)
	{
	}

	///	 Creates a quat defined by rotation axis and angle (in radians).
	/// Assert that \p axis is not normalized
	Quat(const Vec3& axisAngle) noexcept
	{
		const float angleRadians = axisAngle.length();
		if (angleRadians < FLT_EPSILON)
		{
			// No rotation
			x = 0.f;
			y = 0.f;
			z = 0.f;
			w = 1.f;
			return;
		}

		const Vec3 axis = (1.f / angleRadians) * axisAngle;
		const float halfAngle = 0.5f * angleRadians;
		const float sinHalf = std::sin(halfAngle);
		x = axis.x * sinHalf;
		y = axis.y * sinHalf;
		z = axis.z * sinHalf;
		w = std::cos(halfAngle);
	}

	/// Returns the vector part of the quaternion
	[[nodiscard]] Vec3 getVec() const noexcept
	{
		return { x, y, z };
	}

	/// Returns the conjugate quaternion
	[[nodiscard]] Quat getConjugate() const noexcept
	{
		return { -x, -y, -z, w };
	}

	/// Returns the squared length of the quaternion
	[[nodiscard]] float lengthSquared() const noexcept
	{
		return x * x + y * y + z * z + w * w;
	}

	/// Returns the length of the quaternion
	[[nodiscard]] float length() const
	{
		return std::sqrt(lengthSquared());
	}

	/// Checks if the quaternion is normalized
	[[nodiscard]] bool isNormalized() const
	{
		return std::abs(lengthSquared() - 1.0f) < 100.0f * FLT_EPSILON;
	}

	/// Scalar multiplication assignment operator
	Quat& operator*=(float scalar) noexcept
	{
		x *= scalar;
		y *= scalar;
		z *= scalar;
		w *= scalar;
		return *this;
	}
};

/// Scalar multiplication operator
inline [[nodiscard]] Quat operator*(
	float scalar,
	const Quat& quat) noexcept
{
	return Quat(quat) *= scalar;
}

/// Quat multiplication operator
inline [[nodiscard]] Quat operator*(
	const Quat& quatA,
	const Quat& quatB) noexcept
{
	const Vec3 v1 = quatA.getVec();
	const Vec3 v2 = quatB.getVec();
	const Vec3 axis =
		quatA.w * v2 + quatB.w * v1 + cross(v1, v2);

	return {
		axis.x,
		axis.y,
		axis.z,
		quatA.w * quatB.w - dot(v1, v2)
	};
}

/// Quaterion-vector multiplication operator
inline [[nodiscard]] Vec3 operator*(
	const Quat& quat,
	const Vec3& vec) noexcept
{
	const Quat vectorQuat(vec.x, vec.y, vec.z, 0.f);
	const Quat result = (quat * vectorQuat) * quat.getConjugate();
	return result.getVec();
}

/// Returns the rotation matrix corresponding to this quaternion
/// Assert that the quaternion is normalized
inline [[nodiscard]] Mat33 rotationMat(const Quat& q) noexcept
{
	assert(q.isNormalized());

	// Computations used for optimization (less multiplications)
	const float xs = q.x * 2.f;
	const float ys = q.y * 2.f;
	const float zs = q.z * 2.f;
	const float wxs = q.w * xs;
	const float wys = q.w * ys;
	const float wzs = q.w * zs;
	const float xxs = q.x * xs;
	const float xys = q.x * ys;
	const float xzs = q.x * zs;
	const float yys = q.y * ys;
	const float yzs = q.y * zs;
	const float zzs = q.z * zs;

	return {
		{ 1.f - yys - zzs, xys + wzs, xzs - wys },
		{ xys - wzs, 1.f - xxs - zzs, yzs + wxs },
		{ xzs + wys, yzs - wxs, 1.f - xxs - yys } };
}

} // namespace nph
