// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025-2026 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

namespace nph
{

/// Generic matrix template
template <uint16_t R, uint16_t C, typename T = float>
struct Mat;

/// Matrix addition operator
template <uint16_t R, uint16_t C, typename T>
inline [[nodiscard]] Mat<R, C, T> operator+(
	const Mat<R, C, T>& matA,
	const Mat<R, C, T>& matB) noexcept
{
	return Mat<R, C, T>(matA) += matB;
}

/// Matrix subtraction operator
template <uint16_t R, uint16_t C, typename T>
inline [[nodiscard]] Mat<R, C, T> operator-(
	const Mat<R, C, T>& matA,
	const Mat<R, C, T>& matB) noexcept
{
	return Mat<R, C, T>(matA) -= matB;
}

} // namespace nph
