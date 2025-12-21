// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

// Includes
#include <cassert>
#include <cmath>

namespace nph
{

/// Generic vector template
template <uint16_t D>
struct Vec;

/// Vector subtraction operator
template <uint16_t D>
[[nodiscard]] Vec<D> operator-(
	const Vec<D>& vecA,
	const Vec<D>& vecB) noexcept
{
	return Vec<D>(vecA) -= vecB;
}

/// Vector addition operator
template <uint16_t D>
[[nodiscard]] Vec<D> operator+(
	const Vec<D>& vecA,
	const Vec<D>& vecB) noexcept
{
	return Vec<D>(vecA) += vecB;
}

/// Scalar multiplication operator
///	\note the vector-scalar operator is
/// intentionally not defined to avoid
/// optimization flaws like 2.0f * vec * 3.0f
template <uint16_t D>
[[nodiscard]] Vec<D> operator*(
	float scalar,
	const Vec<D>& vec) noexcept
{
	return Vec<D>(vec) *= scalar;
}

} // namespace nph
