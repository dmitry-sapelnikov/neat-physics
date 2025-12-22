// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

// Includes
#include <cassert>

namespace nph
{

/// RGB color
struct Color
{
	/// Red channel
	float r{ 0.0f };

	/// Green channel
	float g{ 0.0f };

	/// Blue channel
	float b{ 0.0f };

	/// Indexing operator (const version)
	const float& operator[](size_t index) const
	{
		assert(index < 3);
		return *(&r + index);
	}

	/// Indexing operator
	float& operator[](size_t index)
	{
		assert(index < 3);
		return *(&r + index);
	}
};

} // namespace nph
