// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

// Includes
#include "neat_physics/math/Vec2.h"

namespace nph
{

/// Generic matrix template
template <uint16_t R, uint16_t C>
struct Mat;

/// 2x2 matrix, column major
template <>
struct Mat<2, 2>
{
	/// Column #1, intentionally uninitialized
	Vec2 col1;

	/// Column #2, intentionally uninitialized
	Vec2 col2;

	/// Default constructor (no initialization)
	Mat() noexcept = default;

	/// Constructor with columns
	constexpr Mat(const Vec2& inCol1, const Vec2& inCol2) noexcept :
		col1(inCol1), col2(inCol2)
	{
	}

	/// Returns the transposed matrix
	[[nodiscard]] Mat getTransposed() const noexcept
	{
		return { { col1.x, col2.x }, { col1.y, col2.y } };
	}

	/// Column access operator (const version)
	[[nodiscard]] const Vec2& operator[](int index) const noexcept
	{
		assert(index < 2);
		return *(&col1 + index);
	}

	/// Column access operator (non-const version)
	[[nodiscard]] Vec2& operator[](int index) noexcept
	{
		assert(index < 2);
		return *(&col1 + index);
	}

	/// Addition assignment operator
	Mat& operator+=(const Mat& other) noexcept
	{
		col1 += other.col1;
		col2 += other.col2;
		return *this;
	}

	/// Subtraction assignment operator
	Mat& operator-=(const Mat& other) noexcept
	{
		col1 -= other.col1;
		col2 -= other.col2;
		return *this;
	}
};

using Mat22 = Mat<2, 2>;

/// Matrix addition operator
template <uint16_t R, uint16_t C>
inline [[nodiscard]] Mat<R, C> operator+(
	const Mat<R, C>& matA,
	const Mat<R, C>& matB) noexcept
{
	return Mat<R, C>(matA) += matB;
}

/// Matrix subtraction operator
template <uint16_t R, uint16_t C>
inline [[nodiscard]] Mat<R, C> operator-(
	const Mat<R, C>& matA,
	const Mat<R, C>& matB) noexcept
{
	return Mat<R, C>(matA) -= matB;
}

/// Matrix-vector multiplication operator
inline [[nodiscard]] Vec2 operator*(
	const Mat22& mat,
	const Vec2& vec) noexcept
{
	return {
		mat.col1.x * vec.x + mat.col2.x * vec.y,
		mat.col1.y * vec.x + mat.col2.y * vec.y };
}

/// Matrix-matrix multiplication operator
inline [[nodiscard]] Mat22 operator*(
	const Mat22& matA,
	const Mat22& matB) noexcept
{
	return { matA * matB.col1, matA * matB.col2 };
}

/// Component-wise absolute value of a matrix
inline [[nodiscard]] Mat22 abs(const Mat22& mat) noexcept
{
	return { abs(mat.col1), abs(mat.col2) };
}

/// Returns a rotation matrix for a given angle in radians
inline [[nodiscard]] Mat22 rotationMat(float angleRad) noexcept
{
	const float c = std::cos(angleRad);
	const float s = std::sin(angleRad);
	return { {c, s}, {-s, c} };
}

} // namespace nph
