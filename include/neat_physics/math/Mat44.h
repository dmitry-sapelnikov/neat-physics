// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

// Includes
#include <memory>
#include "neat_physics/math/Mat33.h"

namespace nph
{

/// Represents a column-major 4x4 matrix
template <>
struct Mat<4, 4>
{
public:
	// Constructor
	constexpr Mat(
		float m00 = 0, float m01 = 0, float m02 = 0, float m03 = 0,
		float m10 = 0, float m11 = 0, float m12 = 0, float m13 = 0,
		float m20 = 0, float m21 = 0, float m22 = 0, float m23 = 0,
		float m30 = 0, float m31 = 0, float m32 = 0, float m33 = 0) noexcept
	{
		// The 1st row
		m[0][0] = m00;
		m[1][0] = m01;
		m[2][0] = m02;
		m[3][0] = m03;

		// The 2nd row
		m[0][1] = m10;
		m[1][1] = m11;
		m[2][1] = m12;
		m[3][1] = m13;

		// The 3rd row
		m[0][2] = m20;
		m[1][2] = m21;
		m[2][2] = m22;
		m[3][2] = m23;

		// The 4th row
		m[0][3] = m30;
		m[1][3] = m31;
		m[2][3] = m32;
		m[3][3] = m33;
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
				float v = 0;
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
	Vec3 operator*(const Vec3& v) const noexcept
	{
		Vec3 u(
			m[0][0] * v.x + m[1][0] * v.y + m[2][0] * v.z + m[3][0],
			m[0][1] * v.x + m[1][1] * v.y + m[2][1] * v.z + m[3][1],
			m[0][2] * v.x + m[1][2] * v.y + m[2][2] * v.z + m[3][2]);

		float w = m[0][3] * v.x + m[1][3] * v.y + m[2][3] * v.z + m[3][3];
		return (1.f / w) * u;
	}

	/// *= operator
	Mat& operator*=(float f) noexcept
	{
		for (float* value = data(); value < data() + 16; value++)
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
	Vec3 getAxis(uint32_t i) const noexcept
	{
		assert(i < 3);
		const auto& col = m[i];
		return { col[0], col[1], col[2] };
	}

	/// Sets the axis at the specified index
	void setAxis(uint32_t i, const Vec3& v) noexcept
	{
		assert(i < 3);
		auto& col = m[i];
		col[0] = v.x;
		col[1] = v.y;
		col[2] = v.z;
	}

	/// Returns a pointer to the matrix data (non-const version)
	float* data() noexcept
	{
		return m[0];
	}

	/// Returns a pointer to the matrix data (const version)
	const float* data() const noexcept
	{
		return m[0];
	}

	/// Returns the element at the specified row and column
	float operator()(uint32_t row, uint32_t col) const noexcept
	{
		assert(row < 4 && col < 4);
		return m[col][row];
	}

	/// Returns the element at the specified row and column
	float& operator()(uint32_t row, uint32_t col) noexcept
	{
		assert(row < 4 && col < 4);
		return m[col][row];
	}

	/// Returns the inverse of the matrix
	Mat getInverse() const;

	/// Extracts the translation component from the matrix
	Vec3 getTranslation() const noexcept
	{
		return { m[3][0], m[3][1], m[3][2] };
	}

	/// Returns the identity matrix
	static constexpr Mat identity() noexcept;

	/// Returns a 4x4 translation matrix
	static Mat translationMatrix(const Vec3& v) noexcept;

	/// Returns a 4x4 rotation matrix
	static Mat rotationMatrix(const Vec3& axisAngle) noexcept;

	/// Returns a 4x4 scale matrix
	static Mat scaleMatrix(const Vec3& s) noexcept;

	static Mat transformMatrix(
		const Vec3& position,
		const Vec3& axisAngle = { 0.f, 0.f, 0.f },
		const Vec3& scale = { 1.f, 1.f, 1.f }) noexcept;

	/// Returns a 4x4 look-at matrix
	static Mat lookAtMatrix(
		const Vec3& position,
		const Vec3& target,
		const Vec3& up) noexcept;

	/// Returns a 4x4 perspective projection matrix
	static Mat perspectiveProjectionMatrix(
		float fieldOfViewRadians,
		float aspectRatio,
		float nearDistance,
		float farDistance) noexcept;

	/// Returns a 4x4 orthographic projection matrix
	static Mat orthographicProjectionMatrix(
		float width,
		float height,
		float nearDistance,
		float farDistance) noexcept;

private:
	/// The matrix data in column-major order
	float m[4][4];
};

/// 4x4 matrix alias
using Mat44 = Mat<4, 4>;

inline Mat<4, 4> Mat<4, 4>::getInverse() const
{
	const auto& M = *this;

	float d =
		(M(0, 0) * M(1, 1) - M(0, 1) * M(1, 0)) * (M(2, 2) * M(3, 3) - M(2, 3) * M(3, 2)) -
		(M(0, 0) * M(1, 2) - M(0, 2) * M(1, 0)) * (M(2, 1) * M(3, 3) - M(2, 3) * M(3, 1)) +
		(M(0, 0) * M(1, 3) - M(0, 3) * M(1, 0)) * (M(2, 1) * M(3, 2) - M(2, 2) * M(3, 1)) +
		(M(0, 1) * M(1, 2) - M(0, 2) * M(1, 1)) * (M(2, 0) * M(3, 3) - M(2, 3) * M(3, 0)) -
		(M(0, 1) * M(1, 3) - M(0, 3) * M(1, 1)) * (M(2, 0) * M(3, 2) - M(2, 2) * M(3, 0)) +
		(M(0, 2) * M(1, 3) - M(0, 3) * M(1, 2)) * (M(2, 0) * M(3, 1) - M(2, 1) * M(3, 0));

	assert(std::abs(d) >= FLT_EPSILON);

	d = 1.f / d;

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
inline Mat44 operator*(float f, const Mat44& m) noexcept
{
	return Mat44(m) *= f;
}

inline Mat<4, 4> Mat<4, 4>::identity() noexcept
{
	return {
		1.f, 0.f, 0.f, 0.f,
		0.f, 1.f, 0.f, 0.f,
		0.f, 0.f, 1.f, 0.f,
		0.f, 0.f, 0.f, 1.f };
}

inline Mat<4, 4> Mat<4, 4>::translationMatrix(const Vec3& v) noexcept
{
	return {
		1.f, 0.f, 0.f, v.x,
		0.f, 1.f, 0.f, v.y,
		0.f, 0.f, 1.f, v.z,
		0.f, 0.f, 0.f, 1.f };
}

inline Mat<4, 4> Mat<4, 4>::rotationMatrix(const Vec3& axisAngle) noexcept
{
	const float angle = axisAngle.length();
	if (angle < std::numeric_limits<float>::epsilon())
	{
		return identity();
	}
	const Vec3 axis = (1.f / angle) * axisAngle;

	const float sinA = std::sin(angle);
	const float cosA = std::cos(angle);
	return {
		cosA + (1.f - cosA) * axis.x * axis.x,
		(1.f - cosA) * axis.x * axis.y - axis.z * sinA,
		(1.f - cosA) * axis.x * axis.z + axis.y * sinA,
		0.f,

		(1.f - cosA) * axis.x * axis.y + axis.z * sinA,
		cosA + (1.f - cosA) * axis.y * axis.y,
		(1.f - cosA) * axis.y * axis.z - axis.x * sinA,
		0.f,

		(1.f - cosA) * axis.x * axis.z - axis.y * sinA,
		(1.f - cosA) * axis.y * axis.z + axis.x * sinA,
		cosA + (1.f - cosA) * axis.z * axis.z,
		0.f,

		0.f,
		0.f,
		0.f,
		1.f };
}

inline Mat<4, 4> Mat<4, 4>::scaleMatrix(const Vec3& s) noexcept
{
	return {
		s.x, 0.f, 0.f, 0.f,
		0.f, s.y, 0.f, 0.f,
		0.f, 0.f, s.z, 0.f,
		0.f, 0.f, 0.f, 1.f };
}

inline Mat<4, 4> Mat<4, 4>::transformMatrix(
	const Vec3& position,
	const Vec3& axisAngle,
	const Vec3& scale) noexcept
{
	return translationMatrix(position) *
		rotationMatrix(axisAngle) *
		scaleMatrix(scale);
}

inline Mat<4, 4> Mat<4, 4>::lookAtMatrix(
	const Vec3& position,
	const Vec3& target,
	const Vec3& up) noexcept
{
	// Row-major gluLookAt
	const Vec3 f = (target - position).getNormalized();
	const Vec3 s = cross(f, up).getNormalized();
	const Vec3 u = cross(s, f);
	return {
		s.x, s.y, s.z, -dot(s, position),
		u.x, u.y, u.z, -dot(u, position),
		-f.x, -f.y, -f.z, dot(f, position),
		0.f, 0.f, 0.f, 1.f };
}

inline Mat<4, 4> Mat<4, 4>::perspectiveProjectionMatrix(
	float fieldOfViewRadians,
	float aspectRatio,
	float nearDistance,
	float farDistance) noexcept
{
	assert(fieldOfViewRadians > FLT_EPSILON);
	assert(aspectRatio > FLT_EPSILON);
	assert(nearDistance > FLT_EPSILON);
	assert(farDistance > nearDistance);

	const float top = nearDistance * std::tan(fieldOfViewRadians / 2.f);
	const float bottom = -top;
	const float left = bottom * aspectRatio;
	const float right = top * aspectRatio;

	const float fx = 2.f * nearDistance / (right - left);
	const float fy = 2.f * nearDistance / (top - bottom);
	const float fz = -(farDistance + nearDistance) / (farDistance - nearDistance);
	const float fw = -2.f * farDistance * nearDistance / (farDistance - nearDistance);

	// Compute the projection matrix
	return {
		fx, 0.f, 0.f, 0.f,
		0.f, fy, 0.f, 0.f,
		0.f, 0.f, fz, fw,
		0.f, 0.f, -1.f, 0.f };
}

inline Mat44 Mat<4, 4>::orthographicProjectionMatrix(
	float width,
	float height,
	float nearDistance,
	float farDistance) noexcept
{
	assert(width > FLT_EPSILON);
	assert(height > FLT_EPSILON);
	assert(nearDistance > FLT_EPSILON);
	assert(farDistance > nearDistance);

	const float fx = 2.f / width;
	const float fy = 2.f / height;
	const float fz = -2.f / (farDistance - nearDistance);
	const float fw = -(farDistance + nearDistance) / (farDistance - nearDistance);

	// Compute the projection matrix
	return {
		fx, 0.f, 0.f, 0.f,
		0.f, fy, 0.f, 0.f,
		0.f, 0.f, fz, fw,
		0.f, 0.f, 0.f, 1.f };
}

} // namespace nph
