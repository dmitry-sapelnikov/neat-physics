// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025-2026 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT
#pragma once

// Includes
#include "neat_physics/math/Mat.h"
#include "neat_physics/math/Vec3.h"

namespace nph
{

/// 3x3 matrix, column major
template <typename T>
struct Mat<3, 3, T>
{
	/// Column #1, intentionally uninitialized
	Vec<3, T> col1;

	/// Column #2, intentionally uninitialized
	Vec<3, T> col2;

	/// Column #3, intentionally uninitialized
	Vec<3, T> col3;

	/// Default constructor (no initialization)
	Mat() noexcept = default;

	/// Constructor with columns
	constexpr Mat(
		const Vec<3, T>& inCol1,
		const Vec<3, T>& inCol2,
		const Vec<3, T>& inCol3) noexcept :

		col1(inCol1),
		col2(inCol2),
		col3(inCol3)
	{
	}

	/// Returns the transposed matrix
	[[nodiscard]] constexpr Mat getTransposed() const noexcept
	{
		return {
			{ col1.x, col2.x, col3.x },
			{ col1.y, col2.y, col3.y },
			{ col1.z, col2.z, col3.z } };
	}

	/// Returns the inverted matrix
	[[nodiscard]] Mat getInverse() const noexcept;

	/// Column access operator (const version)
	[[nodiscard]] const Vec<3, T>& operator[](uint32_t index) const noexcept
	{
		assert(index < 3);
		return *(&col1 + index);
	}

	/// Column access operator (non-const version)
	[[nodiscard]] Vec<3, T>& operator[](uint32_t index) noexcept
	{
		assert(index < 3);
		return *(&col1 + index);
	}

	/// Scalar multiplication operator
	[[nodiscard]] Mat& operator*=(T scalar) noexcept
	{
		col1 *= scalar;
		col2 *= scalar;
		col3 *= scalar;
		return *this;
	}

	/// Addition assignment operator
	Mat& operator+=(const Mat& other) noexcept
	{
		col1 += other.col1;
		col2 += other.col2;
		col3 += other.col3;
		return *this;
	}

	/// Subtraction assignment operator
	Mat& operator-=(const Mat& other) noexcept
	{
		col1 -= other.col1;
		col2 -= other.col2;
		col3 -= other.col3;
		return *this;
	}
};

/// Matrix-scalar multiplication operator
template <typename T>
inline [[nodiscard]] Mat<3, 3, T> operator*(
	T scalar,
	const Mat<3, 3, T>& mat) noexcept
{
	return {
		scalar * mat.col1,
		scalar * mat.col2,
		scalar * mat.col3 };
}

/// Matrix addition operator
template <typename T>
inline [[nodiscard]] Mat<3, 3, T> operator+(
	const Mat<3, 3, T>& matA,
	const Mat<3, 3, T>& matB) noexcept
{
	return {
		matA.col1 + matB.col1,
		matA.col2 + matB.col2,
		matA.col3 + matB.col3 };
}

/// Matrix subtraction operator
template <typename T>
inline [[nodiscard]] Mat<3, 3, T> operator-(
	const Mat<3, 3, T>& matA,
	const Mat<3, 3, T>& matB) noexcept
{
	return {
		matA.col1 - matB.col1,
		matA.col2 - matB.col2,
		matA.col3 - matB.col3 };
}

/// Matrix-vector multiplication operator
template <typename T>
inline [[nodiscard]] Vec<3, T> operator*(
	const Mat<3, 3, T>& mat,
	const Vec<3, T>& vec) noexcept
{
	// This works faster than 3 dot() calls
	return {
		mat.col1.x * vec.x + mat.col2.x * vec.y + mat.col3.x * vec.z,
		mat.col1.y * vec.x + mat.col2.y * vec.y + mat.col3.y * vec.z,
		mat.col1.z * vec.x + mat.col2.z * vec.y + mat.col3.z * vec.z
	};
}

/// Matrix-matrix multiplication operator
template <typename T>
inline [[nodiscard]] Mat<3, 3, T> operator*(
	const Mat<3, 3, T>& matA,
	const Mat<3, 3, T>& matB) noexcept
{
	return { matA * matB.col1, matA * matB.col2, matA * matB.col3 };
}

/// Component-wise absolute value of a matrix
template <typename T>
inline [[nodiscard]] Mat<3, 3, T> abs(const Mat<3, 3, T>& mat) noexcept
{
	return { abs(mat.col1), abs(mat.col2), abs(mat.col3) };
}

template <typename T>
inline [[nodiscard]] Mat<3, 3, T> Mat<3, 3, T>::getInverse() const noexcept
{
	const Mat& m = *this;
	// Compute the determinant of the matrix
	const T determinant =
		m[0][0] * (m[1][1] * m[2][2] - m[2][1] * m[1][2]) -
		m[0][1] * (m[1][0] * m[2][2] - m[2][0] * m[1][2]) +
		m[0][2] * (m[1][0] * m[2][1] - m[2][0] * m[1][1]);

	assert(std::abs(determinant) > 0.0);

	const T invDeterminant = 1.f / determinant;
	return invDeterminant * Mat(
		{
			 (m[1][1] * m[2][2] - m[2][1] * m[1][2]),
			-(m[1][0] * m[2][2] - m[2][0] * m[1][2]),
			 (m[1][0] * m[2][1] - m[2][0] * m[1][1])
		},
		{
			-(m[0][1] * m[2][2] - m[2][1] * m[0][2]),
			 (m[0][0] * m[2][2] - m[2][0] * m[0][2]),
			-(m[0][0] * m[2][1] - m[2][0] * m[0][1])
		},
		{
			 (m[0][1] * m[1][2] - m[0][2] * m[1][1]),
			-(m[0][0] * m[1][2] - m[1][0] * m[0][2]),
			 (m[0][0] * m[1][1] - m[0][1] * m[1][0])
		});
}

/// 3x3 matrix alias
using Mat33 = Mat<3, 3, float>;

} // namespace nph
