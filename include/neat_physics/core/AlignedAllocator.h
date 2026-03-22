// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025-2026 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

// Includes
#include <bit>
#include <cstdlib>
#include <new>

#include "neat_physics/core/Platform.h"
#ifdef NPH_COMPILER_MSVC
#include <malloc.h>
#endif
#include "neat_physics/core/Check.h"

namespace nph
{

/// A simple aligned allocator class
template <typename T, std::size_t Alignment>
struct AlignedAllocator
{
	static_assert(
		std::has_single_bit(Alignment),
		"Alignment must be a power of 2");

	/// Value type
	using value_type = T;

	/// Default constructor
	AlignedAllocator() noexcept = default;

	/// Copy constructor
	AlignedAllocator(const AlignedAllocator&) noexcept = default;

	/// Other type constructor
	template <typename U>
	AlignedAllocator(const AlignedAllocator<U, Alignment>&) noexcept
	{
	}

	/// Rebind allocator to type U
	template <typename U>
	struct rebind
	{
		using other = AlignedAllocator<U, Alignment>;
	};

	/// Allocates n elements of type T with specific alignment
	T* allocate(std::size_t n)
	{
		// Calculate size and ensure it's a multiple of Alignment
		const std::size_t size = n * sizeof(T);
		const std::size_t alignedSize = (size + Alignment - 1) & (~(Alignment - 1));
		NPH_ASSERT(alignedSize % Alignment == 0);

#ifdef NPH_COMPILER_MSVC
		void* ptr = _aligned_malloc(alignedSize, Alignment);
#else
		void* ptr = std::aligned_alloc(Alignment, alignedSize);
#endif
		if (!ptr)
		{
			throw std::bad_alloc();
		}
		return static_cast<T*>(ptr);
	}

	/// Deallocates memory allocated by allocate
	void deallocate(T* ptr, std::size_t) noexcept
	{
#ifdef NPH_COMPILER_MSVC
		_aligned_free(ptr);
#else
		std::free(ptr);
#endif
	}
};

/// Equality operator for 2 allocators: defined only for allocators
/// with equal alignment, always returns true
template <typename T, typename U, std::size_t Alignment>
bool operator==(
	const AlignedAllocator<T, Alignment>&,
	const AlignedAllocator<U, Alignment>&) noexcept
{
	// Allocators with the same alignment can deallocate each other's memory
	return true;
}

/// Inequality operator for 2 allocators: defined only for allocators
/// with equal alignment, always returns false
template <typename T, typename U, std::size_t Alignment>
bool operator!=(
	const AlignedAllocator<T, Alignment>&,
	const AlignedAllocator<U, Alignment>&) noexcept
{
	return false;
}

} // namespace nph
