// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025-2026 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

// Includes
#include <memory>
#include "neat_physics/math/Mat33.h"

namespace nph
{

/// Represents a column-major 4x4 matrix
template <typename T>
struct Mat<4, 4, T>
{
public:
	// Constructor
	constexpr Mat(
		T m00 = T(0), T m01 = T(0), T m02 = T(0), T m03 = T(0),
		T m10 = T(0), T m11 = T(0), T m12 = T(0), T m13 = T(0),
		T m20 = T(0), T m21 = T(0), T m22 = T(0), T m23 = T(0),
		T m30 = T(0), T m31 = T(0), T m32 = T(0), T m33 = T(0)) noexcept :

		m{ { m00, m10, m20, m30 },
		   { m01, m11, m21, m31 },
		   { m02, m12, m22, m32 },
		   { m03, m13, m23, m33 } }
	{
	}

	// += operator
	Mat& operator+=(const Mat& n) noexcept
	{
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				m[i][j] += n.m[i][j];
			}
		}
		return *this;
	}

	// + operator
	Mat operator+(const Mat& n) const noexcept
	{
		return Mat(*this) += n;
	}

	/// -= operator
	Mat& operator-=(const Mat& n) noexcept
	{
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				m[i][j] -= n.m[i][j];
			}
		}
		return *this;
	}

	/// - operator
	Mat operator-(const Mat& n) const noexcept
	{
		return Mat(*this) -= n;
	}

	/// Matrix multiplication operator
	Mat operator*(const Mat& n) const noexcept
	{
		Mat result;
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				T v = 0;
				for (int k = 0; k < 4; k++)
				{
					v += m[k][i] * n.m[j][k];
				}
				result.m[j][i] = v;
			}
		}
		return result;
	}

	/// Matrix-vector multiplication operator
	Vec<3, T> operator*(const Vec<3, T>& v) const noexcept
	{
		Vec<3, T> u(
			m[0][0] * v.x + m[1][0] * v.y + m[2][0] * v.z + m[3][0],
			m[0][1] * v.x + m[1][1] * v.y + m[2][1] * v.z + m[3][1],
			m[0][2] * v.x + m[1][2] * v.y + m[2][2] * v.z + m[3][2]);

		T w = m[0][3] * v.x + m[1][3] * v.y + m[2][3] * v.z + m[3][3];
		return (T(1) / w) * u;
	}

	/// *= operator
	Mat& operator*=(T f) noexcept
	{
		for (T* value = data(); value < data() + 16; value++)
		{
			*value *= f;
		}
		return *this;
	}

	// - operator
	Mat operator-() const noexcept
	{
		return {
			-m[0][0], -m[1][0], -m[2][0], -m[3][0],
			-m[0][1], -m[1][1], -m[2][1], -m[3][1],
			-m[0][2], -m[1][2], -m[2][2], -m[3][2],
			-m[0][3], -m[1][3], -m[2][3], -m[3][3] };
	}

	/// Returns the transpose matrix
	Mat getTranspose() const noexcept
	{
		return {
			m[0][0], m[0][1], m[0][2], m[0][3],
			m[1][0], m[1][1], m[1][2], m[1][3],
			m[2][0], m[2][1], m[2][2], m[2][3],
			m[3][0], m[3][1], m[3][2], m[3][3] };
	}

	/// Returns the axis at the specified index
	Vec<3, T> getAxis(uint32_t i) const noexcept
	{
		assert(i < 3);
		const auto& col = m[i];
		return { col[0], col[1], col[2] };
	}

	/// Sets the axis at the specified index
	void setAxis(uint32_t i, const Vec<3, T>& v) noexcept
	{
		assert(i < 3);
		auto& col = m[i];
		col[0] = v.x;
		col[1] = v.y;
		col[2] = v.z;
	}

	/// Returns a pointer to the matrix data (non-const version)
	T* data() noexcept
	{
		return m[0];
	}

	/// Returns a pointer to the matrix data (const version)
	const T* data() const noexcept
	{
		return m[0];
	}

	/// Returns the element at the specified row and column
	T operator()(uint32_t row, uint32_t col) const noexcept
	{
		assert(row < 4 && col < 4);
		return m[col][row];
	}

	/// Returns the element at the specified row and column
	T& operator()(uint32_t row, uint32_t col) noexcept
	{
		assert(row < 4 && col < 4);
		return m[col][row];
	}

	/// Returns the inverse of the matrix
	Mat getInverse() const;

	/// Extracts the translation component from the matrix
	Vec<3, T> getTranslation() const noexcept
	{
		return { m[3][0], m[3][1], m[3][2] };
	}

	/// Returns the identity matrix
	static Mat identity() noexcept;

	/// Returns a 4x4 translation matrix
	static Mat translationMatrix(const Vec<3, T>& v) noexcept;

	/// Returns a 4x4 rotation matrix
	static Mat rotationMatrix(const Vec<3, T>& axisAngle) noexcept;

	/// Returns a 4x4 scale matrix
	static Mat scaleMatrix(const Vec<3, T>& s) noexcept;

	static Mat transformMatrix(
		const Vec<3, T>& position,
		const Vec<3, T>& axisAngle = { T(0), T(0), T(0) },
		const Vec<3, T>& scale = { T(1), T(1), T(1) }) noexcept;

	/// Returns a 4x4 look-at matrix
	static Mat lookAtMatrix(
		const Vec<3, T>& position,
		const Vec<3, T>& target,
		const Vec<3, T>& up) noexcept;

	/// Returns a 4x4 perspective projection matrix
	static Mat perspectiveProjectionMatrix(
		T fieldOfViewRadians,
		T aspectRatio,
		T nearDistance,
		T farDistance) noexcept;

	/// Returns a 4x4 orthographic projection matrix
	static Mat orthographicProjectionMatrix(
		T width,
		T height,
		T nearDistance,
		T farDistance) noexcept;

private:
	/// The matrix data in column-major order
	T m[4][4];
};

template <typename T>
inline Mat<4, 4, T> Mat<4, 4, T>::getInverse() const
{
	const auto& M = *this;

	T d =
		(M(0, 0) * M(1, 1) - M(0, 1) * M(1, 0)) * (M(2, 2) * M(3, 3) - M(2, 3) * M(3, 2)) -
		(M(0, 0) * M(1, 2) - M(0, 2) * M(1, 0)) * (M(2, 1) * M(3, 3) - M(2, 3) * M(3, 1)) +
		(M(0, 0) * M(1, 3) - M(0, 3) * M(1, 0)) * (M(2, 1) * M(3, 2) - M(2, 2) * M(3, 1)) +
		(M(0, 1) * M(1, 2) - M(0, 2) * M(1, 1)) * (M(2, 0) * M(3, 3) - M(2, 3) * M(3, 0)) -
		(M(0, 1) * M(1, 3) - M(0, 3) * M(1, 1)) * (M(2, 0) * M(3, 2) - M(2, 2) * M(3, 0)) +
		(M(0, 2) * M(1, 3) - M(0, 3) * M(1, 2)) * (M(2, 0) * M(3, 1) - M(2, 1) * M(3, 0));

	assert(std::abs(d) >= std::numeric_limits<T>::epsilon());

	d = T(1) / d;

	Mat out;
	out(0, 0) = d * (M(1, 1) * (M(2, 2) * M(3, 3) - M(2, 3) * M(3, 2)) +
		M(1, 2) * (M(2, 3) * M(3, 1) - M(2, 1) * M(3, 3)) +
		M(1, 3) * (M(2, 1) * M(3, 2) - M(2, 2) * M(3, 1)));

	out(0, 1) = d * (M(2, 1) * (M(0, 2) * M(3, 3) - M(0, 3) * M(3, 2)) +
		M(2, 2) * (M(0, 3) * M(3, 1) - M(0, 1) * M(3, 3)) +
		M(2, 3) * (M(0, 1) * M(3, 2) - M(0, 2) * M(3, 1)));

	out(0, 2) = d * (M(3, 1) * (M(0, 2) * M(1, 3) - M(0, 3) * M(1, 2)) +
		M(3, 2) * (M(0, 3) * M(1, 1) - M(0, 1) * M(1, 3)) +
		M(3, 3) * (M(0, 1) * M(1, 2) - M(0, 2) * M(1, 1)));

	out(0, 3) = d * (M(0, 1) * (M(1, 3) * M(2, 2) - M(1, 2) * M(2, 3)) +
		M(0, 2) * (M(1, 1) * M(2, 3) - M(1, 3) * M(2, 1)) +
		M(0, 3) * (M(1, 2) * M(2, 1) - M(1, 1) * M(2, 2)));

	out(1, 0) = d * (M(1, 2) * (M(2, 0) * M(3, 3) - M(2, 3) * M(3, 0)) +
		M(1, 3) * (M(2, 2) * M(3, 0) - M(2, 0) * M(3, 2)) +
		M(1, 0) * (M(2, 3) * M(3, 2) - M(2, 2) * M(3, 3)));

	out(1, 1) = d * (M(2, 2) * (M(0, 0) * M(3, 3) - M(0, 3) * M(3, 0)) +
		M(2, 3) * (M(0, 2) * M(3, 0) - M(0, 0) * M(3, 2)) +
		M(2, 0) * (M(0, 3) * M(3, 2) - M(0, 2) * M(3, 3)));

	out(1, 2) = d * (M(3, 2) * (M(0, 0) * M(1, 3) - M(0, 3) * M(1, 0)) +
		M(3, 3) * (M(0, 2) * M(1, 0) - M(0, 0) * M(1, 2)) +
		M(3, 0) * (M(0, 3) * M(1, 2) - M(0, 2) * M(1, 3)));

	out(1, 3) = d * (M(0, 2) * (M(1, 3) * M(2, 0) - M(1, 0) * M(2, 3)) +
		M(0, 3) * (M(1, 0) * M(2, 2) - M(1, 2) * M(2, 0)) +
		M(0, 0) * (M(1, 2) * M(2, 3) - M(1, 3) * M(2, 2)));

	out(2, 0) = d * (M(1, 3) * (M(2, 0) * M(3, 1) - M(2, 1) * M(3, 0)) +
		M(1, 0) * (M(2, 1) * M(3, 3) - M(2, 3) * M(3, 1)) +
		M(1, 1) * (M(2, 3) * M(3, 0) - M(2, 0) * M(3, 3)));

	out(2, 1) = d * (M(2, 3) * (M(0, 0) * M(3, 1) - M(0, 1) * M(3, 0)) +
		M(2, 0) * (M(0, 1) * M(3, 3) - M(0, 3) * M(3, 1)) +
		M(2, 1) * (M(0, 3) * M(3, 0) - M(0, 0) * M(3, 3)));

	out(2, 2) = d * (M(3, 3) * (M(0, 0) * M(1, 1) - M(0, 1) * M(1, 0)) +
		M(3, 0) * (M(0, 1) * M(1, 3) - M(0, 3) * M(1, 1)) +
		M(3, 1) * (M(0, 3) * M(1, 0) - M(0, 0) * M(1, 3)));

	out(2, 3) = d * (M(0, 3) * (M(1, 1) * M(2, 0) - M(1, 0) * M(2, 1)) +
		M(0, 0) * (M(1, 3) * M(2, 1) - M(1, 1) * M(2, 3)) +
		M(0, 1) * (M(1, 0) * M(2, 3) - M(1, 3) * M(2, 0)));

	out(3, 0) = d * (M(1, 0) * (M(2, 2) * M(3, 1) - M(2, 1) * M(3, 2)) +
		M(1, 1) * (M(2, 0) * M(3, 2) - M(2, 2) * M(3, 0)) +
		M(1, 2) * (M(2, 1) * M(3, 0) - M(2, 0) * M(3, 1)));

	out(3, 1) = d * (M(2, 0) * (M(0, 2) * M(3, 1) - M(0, 1) * M(3, 2)) +
		M(2, 1) * (M(0, 0) * M(3, 2) - M(0, 2) * M(3, 0)) +
		M(2, 2) * (M(0, 1) * M(3, 0) - M(0, 0) * M(3, 1)));

	out(3, 2) = d * (M(3, 0) * (M(0, 2) * M(1, 1) - M(0, 1) * M(1, 2)) +
		M(3, 1) * (M(0, 0) * M(1, 2) - M(0, 2) * M(1, 0)) +
		M(3, 2) * (M(0, 1) * M(1, 0) - M(0, 0) * M(1, 1)));

	out(3, 3) = d * (M(0, 0) * (M(1, 1) * M(2, 2) - M(1, 2) * M(2, 1)) +
		M(0, 1) * (M(1, 2) * M(2, 0) - M(1, 0) * M(2, 2)) +
		M(0, 2) * (M(1, 0) * M(2, 1) - M(1, 1) * M(2, 0)));

	return out;
}

// * operator
template <typename T>
inline Mat<4, 4, T> operator*(T f, const Mat<4, 4, T>& m) noexcept
{
	return Mat44(m) *= f;
}

template <typename T>
inline Mat<4, 4, T> Mat<4, 4, T>::identity() noexcept
{
	return {
		T(1), T(0), T(0), T(0),
		T(0), T(1), T(0), T(0),
		T(0), T(0), T(1), T(0),
		T(0), T(0), T(0), T(1) };
}

template <typename T>
inline Mat<4, 4, T> Mat<4, 4, T>::translationMatrix(const Vec<3, T>& v) noexcept
{
	return {
		T(1), T(0), T(0), v.x,
		T(0), T(1), T(0), v.y,
		T(0), T(0), T(1), v.z,
		T(0), T(0), T(0), T(1) };
}

template <typename T>
inline Mat<4, 4, T> Mat<4, 4, T>::rotationMatrix(const Vec<3, T>& axisAngle) noexcept
{
	const T angle = axisAngle.length();
	if (angle < std::numeric_limits<T>::epsilon())
	{
		return identity();
	}
	const Vec<3, T> axis = (T(1) / angle) * axisAngle;

	const T sinA = std::sin(angle);
	const T cosA = std::cos(angle);
	return {
		cosA + (T(1) - cosA) * axis.x * axis.x,
		(T(1) - cosA) * axis.x * axis.y - axis.z * sinA,
		(T(1) - cosA) * axis.x * axis.z + axis.y * sinA,
		T(0),

		(T(1) - cosA) * axis.x * axis.y + axis.z * sinA,
		cosA + (T(1) - cosA) * axis.y * axis.y,
		(T(1) - cosA) * axis.y * axis.z - axis.x * sinA,
		T(0),

		(T(1) - cosA) * axis.x * axis.z - axis.y * sinA,
		(T(1) - cosA) * axis.y * axis.z + axis.x * sinA,
		cosA + (T(1) - cosA) * axis.z * axis.z,
		T(0),

		T(0),
		T(0),
		T(0),
		T(1) };
}

template <typename T>
inline Mat<4, 4, T> Mat<4, 4, T>::scaleMatrix(const Vec<3, T>& s) noexcept
{
	return {
		s.x, T(0), T(0), T(0),
		T(0), s.y, T(0), T(0),
		T(0), T(0), s.z, T(0),
		T(0), T(0), T(0), T(1) };
}

template <typename T>
inline Mat<4, 4, T> Mat<4, 4, T>::transformMatrix(
	const Vec<3, T>& position,
	const Vec<3, T>& axisAngle,
	const Vec<3, T>& scale) noexcept
{
	return translationMatrix(position) *
		rotationMatrix(axisAngle) *
		scaleMatrix(scale);
}

template <typename T>
inline Mat<4, 4, T> Mat<4, 4, T>::lookAtMatrix(
	const Vec<3, T>& position,
	const Vec<3, T>& target,
	const Vec<3, T>& up) noexcept
{
	// Row-major gluLookAt
	const Vec<3, T> f = (target - position).getNormalized();
	const Vec<3, T> s = cross(f, up).getNormalized();
	const Vec<3, T> u = cross(s, f);
	return {
		s.x, s.y, s.z, -dot(s, position),
		u.x, u.y, u.z, -dot(u, position),
		-f.x, -f.y, -f.z, dot(f, position),
		T(0), T(0), T(0), T(1) };
}

template <typename T>
inline Mat<4, 4, T> Mat<4, 4, T>::perspectiveProjectionMatrix(
	T fieldOfViewRadians,
	T aspectRatio,
	T nearDistance,
	T farDistance) noexcept
{
	assert(fieldOfViewRadians > std::numeric_limits<T>::epsilon());
	assert(aspectRatio > std::numeric_limits<T>::epsilon());
	assert(nearDistance > std::numeric_limits<T>::epsilon());
	assert(farDistance > nearDistance);

	const T top = nearDistance * std::tan(T(0.5) * fieldOfViewRadians);
	const T bottom = -top;
	const T left = bottom * aspectRatio;
	const T right = top * aspectRatio;

	const T fx = T(2) * nearDistance / (right - left);
	const T fy = T(2) * nearDistance / (top - bottom);
	const T fz = -(farDistance + nearDistance) / (farDistance - nearDistance);
	const T fw = -T(2) * farDistance * nearDistance / (farDistance - nearDistance);

	// Compute the projection matrix
	return {
		fx, T(0), T(0), T(0),
		T(0), fy, T(0), T(0),
		T(0), T(0), fz, fw,
		T(0), T(0), -T(1), T(0) };
}

template <typename T>
inline Mat<4, 4, T> Mat<4, 4, T>::orthographicProjectionMatrix(
	T width,
	T height,
	T nearDistance,
	T farDistance) noexcept
{
	assert(width > std::numeric_limits<T>::epsilon());
	assert(height > std::numeric_limits<T>::epsilon());
	assert(nearDistance > std::numeric_limits<T>::epsilon());
	assert(farDistance > nearDistance);

	const T fx =  T(2) / width;
	const T fy =  T(2) / height;
	const T fz = -T(2) / (farDistance - nearDistance);
	const T fw = -(farDistance + nearDistance) / (farDistance - nearDistance);

	// Compute the projection matrix
	return {
		fx, T(0), T(0), T(0),
		T(0), fy, T(0), T(0),
		T(0), T(0), fz, fw,
		T(0), T(0), T(0), T(1) };
}

/// 4x4 matrix alias
using Mat44 = Mat<4, 4, float>;

} // namespace nph
