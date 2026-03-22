// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025-2026 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

//	Includes
#include "neat_physics/math/Mat33.h"

namespace nph
{
/// Quat in convention (x*i, y*j, z*k, w)
template <typename T = float>
struct QuatT
{
	/// X component
	T x;

	/// Y component
	T y;

	/// Z component
	T z;

	/// W component
	T w;

	/// Default constructor (no initialization)
	QuatT() noexcept = default;

	///	All-components constructor
	constexpr QuatT(
		T inX,
		T inY,
		T inZ,
		T inW) noexcept :

		x(inX),
		y(inY),
		z(inZ),
		w(inW)
	{
	}

	///	 Creates a QuatT defined by rotation axis and angle (in radians).
	/// Assert that \p axis is not normalized
	QuatT(const Vec<3, T>& axisAngle) noexcept
	{
		const T angleRadians = axisAngle.length();
		if (angleRadians < std::numeric_limits<T>::epsilon())
		{
			// No rotation
			x = T(0);
			y = T(0);
			z = T(0);
			w = T(1);
			return;
		}

		const Vec<3, T> axis = (T(1) / angleRadians) * axisAngle;
		const T halfAngle = T(0.5) * angleRadians;
		const T sinHalf = std::sin(halfAngle);
		x = axis.x * sinHalf;
		y = axis.y * sinHalf;
		z = axis.z * sinHalf;
		w = std::cos(halfAngle);
	}

	/// Returns the vector part of the QuatTernion
	[[nodiscard]] Vec<3, T> getVec() const noexcept
	{
		return { x, y, z };
	}

	/// Returns the conjugate QuatTernion
	[[nodiscard]] QuatT getConjugate() const noexcept
	{
		return { -x, -y, -z, w };
	}

	/// Returns the squared length of the QuatTernion
	[[nodiscard]] T lengthSquared() const noexcept
	{
		return x * x + y * y + z * z + w * w;
	}

	/// Returns the length of the QuatTernion
	[[nodiscard]] T length() const
	{
		return std::sqrt(lengthSquared());
	}

	/// Checks if the QuatTernion is normalized
	[[nodiscard]] bool isNormalized() const
	{
		return
			std::abs(lengthSquared() - T(1)) <
			T(100) * std::numeric_limits<T>::epsilon();
	}

	/// Returns a normalized version of the QuatTernion,
	/// or an identity QuatTernion
	/// if the vector length < std::numeric_limits<T>::epsilon()
	[[nodiscard]] QuatT getNormalized() const
	{
		const T len = length();
		if (len < std::numeric_limits<T>::epsilon())
		{
			return { T(0), T(0), T(0), T(1) };
		}

		const T invLen = T(1) / len;
		return {
			x * invLen,
			y * invLen,
			z * invLen,
			w * invLen };
	}

	/// Scalar multiplication assignment operator
	QuatT& operator*=(T scalar) noexcept
	{
		x *= scalar;
		y *= scalar;
		z *= scalar;
		w *= scalar;
		return *this;
	}

	/// Addition assignment operator
	QuatT& operator+=(const QuatT& other) noexcept
	{
		x += other.x;
		y += other.y;
		z += other.z;
		w += other.w;
		return *this;
	}
};

/// Scalar multiplication operator
template <typename T>
inline [[nodiscard]] QuatT<T> operator*(
	T scalar,
	const QuatT<T>& quat) noexcept
{
	return QuatT(quat) *= scalar;
}

/// QuatT addition operator
template <typename T>
inline [[nodiscard]] QuatT<T> operator+(
	const QuatT<T>& quatA,
	const QuatT<T>& quatB) noexcept
{
	return QuatT(quatA) += quatB;
}

/// QuatT multiplication operator
template <typename T>
inline [[nodiscard]] QuatT<T> operator*(
	const QuatT<T>& quatA,
	const QuatT<T>& quatB) noexcept
{
	const Vec<3, T> v1 = quatA.getVec();
	const Vec<3, T> v2 = quatB.getVec();
	const Vec<3, T> axis =
		quatA.w * v2 + quatB.w * v1 + cross(v1, v2);

	return {
		axis.x,
		axis.y,
		axis.z,
		quatA.w * quatB.w - dot(v1, v2)
	};
}

/// QuatTerion-vector multiplication operator
template <typename T>
inline [[nodiscard]] Vec<3, T> operator*(
	const QuatT<T>& quat,
	const Vec<3, T>& vec) noexcept
{
	const QuatT<T> vectorQuat(vec.x, vec.y, vec.z, T(0));
	const QuatT<T> result = (quat * vectorQuat) * quat.getConjugate();
	return result.getVec();
}

/// Returns the rotation matrix corresponding to this QuatTernion
/// Assert that the QuatTernion is normalized
template <typename T>
inline [[nodiscard]] Mat<3, 3, T> rotationMat(const QuatT<T>& q) noexcept
{
	assert(q.isNormalized());

	// Computations used for optimization (less multiplications)
	const T xs = q.x * 2.f;
	const T ys = q.y * 2.f;
	const T zs = q.z * 2.f;
	const T wxs = q.w * xs;
	const T wys = q.w * ys;
	const T wzs = q.w * zs;
	const T xxs = q.x * xs;
	const T xys = q.x * ys;
	const T xzs = q.x * zs;
	const T yys = q.y * ys;
	const T yzs = q.y * zs;
	const T zzs = q.z * zs;

	return {
		{ T(1) - yys - zzs, xys + wzs, xzs - wys },
		{ xys - wzs, T(1) - xxs - zzs, yzs + wxs },
		{ xzs + wys, yzs - wxs, T(1) - xxs - yys } };
}

/// Floating-point Quat alias
using Quat = QuatT<float>;

} // namespace nph
