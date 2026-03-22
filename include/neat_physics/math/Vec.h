// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025-2026 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

// Includes
#include <cassert>
#include <cmath>

namespace nph
{

/// Generic vector template
template <uint16_t D, typename T = float>
struct Vec;

/// Vector subtraction operator
template <uint16_t D, typename T>
[[nodiscard]] Vec<D, T> operator-(
	const Vec<D, T>& vecA,
	const Vec<D, T>& vecB) noexcept
{
	return Vec<D, T>(vecA) -= vecB;
}

/// Vector addition operator
template <uint16_t D, typename T>
[[nodiscard]] Vec<D, T> operator+(
	const Vec<D, T>& vecA,
	const Vec<D, T>& vecB) noexcept
{
	return Vec<D, T>(vecA) += vecB;
}

/// Scalar multiplication operator
///	\note the vector-scalar operator is
/// intentionally not defined to avoid
/// optimization flaws like 2.0f * vec * 3.0f
template <uint16_t D, typename T>
[[nodiscard]] Vec<D, T> operator*(
	float scalar,
	const Vec<D, T>& vec) noexcept
{
	return Vec<D, T>(vec) *= scalar;
}

/// Component-wise vector multiplication operator
template <uint16_t D, typename T>
[[nodiscard]] Vec<D, T> operator*(
	const Vec<D, T>& vecA,
	const Vec<D, T>& vecB) noexcept
{
	return Vec<D, T>(vecA) *= vecB;
}

} // namespace nph
