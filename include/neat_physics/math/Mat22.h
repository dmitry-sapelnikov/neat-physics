// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025-2026 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

// Includes
#include "neat_physics/math/Mat.h"
#include "neat_physics/math/Vec2.h"

namespace nph
{

/// 2x2 matrix, column major
template <typename T>
struct Mat<2, 2, T>
{
	/// Column #1, intentionally uninitialized
	Vec<2, T> col1;

	/// Column #2, intentionally uninitialized
	Vec<2, T> col2;

	/// Default constructor (no initialization)
	Mat() noexcept = default;

	/// Constructor with columns
	constexpr Mat(
		const Vec<2, T>& inCol1,
		const Vec<2, T>& inCol2) noexcept :

		col1(inCol1),
		col2(inCol2)
	{
	}

	/// Returns the transposed matrix
	[[nodiscard]] Mat getTransposed() const noexcept
	{
		return { { col1.x, col2.x }, { col1.y, col2.y } };
	}

	/// Column access operator (const version)
	[[nodiscard]] const Vec<2, T>& operator[](uint32_t index) const noexcept
	{
		assert(index < 2);
		return *(&col1 + index);
	}

	/// Column access operator (non-const version)
	[[nodiscard]] Vec<2, T>& operator[](uint32_t index) noexcept
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

/// Matrix-vector multiplication operator
template <typename T>
inline [[nodiscard]] Vec<2, T> operator*(
	const Mat<2, 2, T>& mat,
	const Vec<2, T>& vec) noexcept
{
	// This works faster than 2 dot() calls
	return {
		mat.col1.x * vec.x + mat.col2.x * vec.y,
		mat.col1.y * vec.x + mat.col2.y * vec.y
	};
}

/// Matrix-matrix multiplication operator
template <typename T>
inline [[nodiscard]] Mat<2, 2, T> operator*(
	const Mat<2, 2, T>& matA,
	const Mat<2, 2, T>& matB) noexcept
{
	return { matA * matB.col1, matA * matB.col2 };
}

/// Component-wise absolute value of a matrix
template <typename T>
inline [[nodiscard]] Mat<2, 2, T> abs(const Mat<2, 2, T>& mat) noexcept
{
	return { abs(mat.col1), abs(mat.col2) };
}

/// Returns a rotation matrix for a given angle in radians
template <typename T>
inline [[nodiscard]] Mat<2, 2, T> rotationMat(T angleRad) noexcept
{
	const T c = std::cos(angleRad);
	const T s = std::sin(angleRad);
	return { {c, s}, {-s, c} };
}

/// 2x2 float matrix alias
using Mat22 = Mat<2, 2, float>;

} // namespace nph
