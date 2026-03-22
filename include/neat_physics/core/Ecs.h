// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025-2026 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

// Includes
#include <algorithm>
#include <array>
#include <cassert>
#include <numeric>
#include <span>
#include "neat_physics/core/AlignedAllocator.h"

/// A minimalistic ECS (Entity-Component-System)
/// implementation with named block storage layout
namespace nph::ecs
{

/// Utility function to compute the ceiling of a division,
/// used for alignment calculations
template <typename T>
constexpr T alignUp(T val, T alignment) noexcept
{
	static_assert(
		std::is_unsigned_v<T> && std::is_integral_v<T>,
		"alignUp is only defined for unsigned integral types");
	assert(std::has_single_bit(alignment) && "Alignment must be a power of 2");
	return ((val + alignment - 1) / alignment) * alignment;
}

/// Named block definition for component storage layout
template<typename NameTag, typename T, size_t Count = 1>
struct Block
{
	static_assert(
		std::is_trivially_copyable_v<T>,
		"Block type must be trivially copyable for memcpy operations");

	/// Name tag type
	using name = NameTag;

	/// Value type
	using type = T;

	/// Number of values in the block
	static constexpr size_t count = Count;

	/// Size of a single element
	static constexpr size_t elementSize = sizeof(T);

	/// Total size of the block in bytes
	static constexpr size_t sizeBytes = sizeof(T) * Count;
};

/// Component storage layout definition using named blocks
template<typename... Blocks>
struct Layout
{
	static_assert(sizeof...(Blocks) > 0, "Layout must have at least one block");

	/// Number of blocks in the layout
	static constexpr size_t blockCount = sizeof...(Blocks);

	/// Tuple of name tags
	using NameTags = std::tuple<typename Blocks::name...>;

	/// Type of block at a given index
	template<size_t Index>
	using Block = std::tuple_element_t<Index, std::tuple<Blocks...>>;

	/// Computes offsets for SoA layout with alignment
	static constexpr std::array<size_t, blockCount>
		computeOffsets(size_t entityCount, size_t alignment) noexcept
	{
		std::array<size_t, blockCount> offsets{};
		size_t current = 0;
		size_t i = 0;

		((offsets[i++] = std::exchange(
			current,
			current + alignUp(Blocks::sizeBytes * entityCount, alignment)
		)), ...);

		return offsets;
	}

	/// Computes block sizes with alignment
	static constexpr std::array<size_t, blockCount>
		computeBlockSizes(size_t entityCount, size_t alignment) noexcept
	{
		std::array<size_t, blockCount> result{};
		size_t i = 0;
		((result[i++] = alignUp(Blocks::sizeBytes * entityCount, alignment)), ...);
		return result;
	}

	/// Finds the block index by name tag
	template<typename NameTag, size_t Index = 0>
	static consteval size_t findBlockIndex()
	{
		static_assert(Index < blockCount, "Named block not found in Layout");
		if constexpr (std::is_same_v<typename Block<Index>::name, NameTag>)
		{
			return Index;
		}
		else
		{
			return findBlockIndex<NameTag, Index + 1>();
		}
	}

	/// Check if this layout contains a block with the specified name tag
	template<typename NameTag>
	static consteval bool hasBlock()
	{
		if constexpr (std::disjunction_v<std::is_same<NameTag, typename Blocks::name>...>)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
};

/// Component class. An alias for a span of component data of type T.
template <typename T>
using Component = std::span<T>;

/// View forward declaration
template<typename LayoutT, bool ConstAccess>
class View;

template <class LayoutT>
using MutableView = View<LayoutT, false>;

template <class LayoutT>
using ConstantView = View<LayoutT, true>;

/// Storage traits
struct StorageTraits
{
	/// Component alignment - cache line size for optimal SIMD performance
	static constexpr size_t COMPONENT_ALIGNMENT = 64;

	/// The default storage capacity
	static constexpr size_t DEFAULT_CAPACITY = 16;

	/// Storage type using aligned allocator for optimal SIMD performance
	using ByteStorage =
		std::vector<std::byte, AlignedAllocator<std::byte, COMPONENT_ALIGNMENT>>;
};

/// Component storage class.
/// For maximum performance, use getComponent<Tag>() to get raw pointers.
/// \note The storage is not thread-safe
/// \note The storage does not initialize new elements on resize,
/// so use with trivially copyable types
template<typename LayoutT>
class Storage
{
public:
	/// Layout type
	using Layout = LayoutT;

	/// Traits type
	using Traits = StorageTraits;

	/// Constructor
	/// \param initialSize Initial number of entities in the storage
	explicit Storage(size_t initialSize = 0) :
		mCapacity(std::max(initialSize, Traits::DEFAULT_CAPACITY)),
		mSize(initialSize),
		mOffsets(Layout::computeOffsets(mCapacity, Traits::COMPONENT_ALIGNMENT)),
		mStorage(computeTotalSize(mCapacity))
	{
	}

	/// Returns the number of entities in the storage
	[[nodiscard]] size_t size() const noexcept
	{
		return mSize;
	}

	/// Returns the current capacity
	[[nodiscard]] size_t capacity() const noexcept
	{
		return mCapacity;
	}

	/// Checks if storage is empty
	[[nodiscard]] bool empty() const noexcept
	{
		return mSize == 0;
	}

	/// Reserves storage for at least the specified number of entities
	void reserve(size_t newCapacity)
	{
		if (newCapacity <= mCapacity)
		{
			return;
		}

		// Allocate new storage
		Traits::ByteStorage newStorage(computeTotalSize(newCapacity));
		const auto newOffsets = Layout::computeOffsets(
			newCapacity,
			Traits::COMPONENT_ALIGNMENT);

		// Copy existing data
		const auto blockSizes = Layout::computeBlockSizes(
			mSize,
			Traits::COMPONENT_ALIGNMENT);

		for (size_t i = 0; i < mOffsets.size(); ++i)
		{
			std::memcpy(
				newStorage.data() + newOffsets[i],
				mStorage.data() + mOffsets[i],
				blockSizes[i]);
		}

		// Update state
		mCapacity = newCapacity;
		mOffsets = newOffsets;
		mStorage = std::move(newStorage);
	}

	/// Resizes the storage (does not initialize new elements)
	void resize(size_t newSize)
	{
		if (newSize > mCapacity)
		{
			reserve(newSize);
		}
		mSize = newSize;
	}

	/// Creates a new entity and returns its index
	[[nodiscard]] size_t create()
	{
		if (mSize == mCapacity)
		{
			reserve(mCapacity * 2);
		}
		return mSize++;
	}

	/// Clears all entities (keeps capacity)
	void clear() noexcept
	{
		mSize = 0;
	}

	/// Gets a component array for a name tag
	/// \note The component array becomes invalid after an entity addition
	template<typename NameTag>
	[[nodiscard]] auto getComponent() const noexcept
	{
		return Component(getComponentPtr<NameTag>(), mSize);
	}

	/// Gets a reference to a specific entity's component
	template<typename NameTag>
	[[nodiscard]] auto& get(size_t entity) const noexcept
	{
		assert(entity < mSize && "Entity index out of bounds");
		constexpr size_t blockInd = Layout::template findBlockIndex<NameTag>();
		using Block = typename Layout::template Block<blockInd>;
		return getComponentPtr<NameTag>()[entity * Block::count];
	}

	/// Gets a pointer to a specific entity's component
	template<typename NameTag>
	[[nodiscard]] auto* getPtr(size_t entity) const noexcept
	{
		assert(entity < mSize && "Entity index out of bounds");
		constexpr size_t blockInd = Layout::template findBlockIndex<NameTag>();
		using Block = typename Layout::template Block<blockInd>;
		return getComponentPtr<NameTag>() + entity * Block::count;
	}

	/// Gets the memory offset for a component array
	template<typename NameTag>
	[[nodiscard]] size_t getOffset() const noexcept
	{
		constexpr size_t blockInd = Layout::template findBlockIndex<NameTag>();
		return mOffsets[blockInd];
	}

	/// Gets all offsets (for debugging/introspection)
	[[nodiscard]] const std::array<size_t, Layout::blockCount>& getOffsets() const noexcept
	{
		return mOffsets;
	}

	/// Creates a constant view with the specified layout
	template <typename ViewLayoutT>
	[[nodiscard]] ConstantView<ViewLayoutT> createConstantView() const
	{
		return createView<ViewLayoutT, true>();
	}

	/// Creates a non-const view with the specified layout
	template <typename ViewLayoutT>
	[[nodiscard]] MutableView<ViewLayoutT> createMutableView() const
	{
		return createView<ViewLayoutT, false>();
	}

private:
	/// Computes total size needed for the storage buffer
	[[nodiscard]] static size_t computeTotalSize(size_t entityCount)
	{
		const auto blockSizes = Layout::computeBlockSizes(entityCount, Traits::COMPONENT_ALIGNMENT);
		size_t result = std::accumulate(blockSizes.begin(), blockSizes.end(), size_t{ 0 });
		assert(result >= blockSizes[0] && "Integer overflow in total size calculation");
		return result;
	}

	/// Gets a const pointer to the component array
	template<typename NameTag>
	[[nodiscard]] auto* getComponentPtr() const noexcept
	{
		constexpr size_t blockInd = Layout::template findBlockIndex<NameTag>();
		using Block = typename Layout::template Block<blockInd>;
		return reinterpret_cast<typename Block::type*>(
			const_cast<std::byte*>(mStorage.data()) + mOffsets[blockInd]);
	}

	/// Creates a const / non-const view with the specified layout
	template <class ViewLayoutT, bool ConstAccess>
	[[nodiscard]] View<ViewLayoutT, ConstAccess> createView() const;

	/// Allocated capacity
	size_t mCapacity;

	/// Current number of entities
	size_t mSize;

	/// Component array offsets
	std::array<size_t, Layout::blockCount> mOffsets;

	/// Raw storage buffer
	Traits::ByteStorage mStorage;
};

/// View class providing typed access to components based on a specific layout
template<typename LayoutT, bool ConstAccess>
class View
{
public:
	using ComponentIndices = std::array<size_t, LayoutT::blockCount>;

	/// Returns a component array for a name tag
	template<typename NameTag>
	[[nodiscard]] auto getComponent() const noexcept
	{
		if constexpr (ConstAccess)
		{
			return Component(getConstComponentPtr<NameTag>(), *mSize);
		}
		else
		{
			return Component(getComponentPtr<NameTag>(), *mSize);
		}
	}

	/// Returns the number of entities in the view (same as storage size)
	[[nodiscard]] size_t size() const noexcept
	{
		return *mSize;
	}

	/// Gets a pointer to a specific entity's component
	template<typename NameTag>
	[[nodiscard]] auto& get(size_t entity) const noexcept
	{
		assert(entity < *mSize && "Entity index out of bounds");
		return getComponentPtr<NameTag>()[entity * Block::count];
	}

private:
	/// Only Storage can create views
	template<typename StorageLayoutT>
	friend class Storage;

	/// Private constructor used by Storage to create views
	View(
		const StorageTraits::ByteStorage& byteStorage,
		const size_t* componentOffsets,
		const size_t& size,
		const ComponentIndices& componentIndices) :

		mByteStorage(&byteStorage),
		mComponentOffsets(componentOffsets),
		mSize(&size),
		mComponentIndices(componentIndices)
	{
	}

	/// Returns a raw pointer to the component array for a given name tag
	template<typename NameTag>
	[[nodiscard]] auto* getComponentPtr() const noexcept
	{
		constexpr size_t blockInd = LayoutT::template findBlockIndex<NameTag>();
		using Block = typename LayoutT::template Block<blockInd>;
		return reinterpret_cast<typename Block::type*>(
			const_cast<std::byte*>(mByteStorage->data()) +
			mComponentOffsets[mComponentIndices[blockInd]]);
	}

	/// Returns a raw const pointer to the component array for a given name tag
	template<typename NameTag>
	[[nodiscard]] auto* getConstComponentPtr() const noexcept
	{
		constexpr size_t blockInd = LayoutT::template findBlockIndex<NameTag>();
		using Block = typename LayoutT::template Block<blockInd>;
		return reinterpret_cast<const typename Block::type*>(
			mByteStorage->data() +
			mComponentOffsets[mComponentIndices[blockInd]]);
	}

	/// Byte storage pointer
	const StorageTraits::ByteStorage* mByteStorage;

	/// Component offsets pointer
	const size_t* mComponentOffsets;

	/// Size reference (for bounds checking)
	const size_t* mSize;

	/// Component indices in the component offsets array
	ComponentIndices mComponentIndices;
};

template <class LayoutT>
template <class ViewLayoutT, bool ConstAccess>
inline [[nodiscard]] View<ViewLayoutT, ConstAccess> Storage<LayoutT>::createView() const
{
	// Check that the LayoutT contains all blocks required by ViewLayoutT
	constexpr bool allBlocksPresent = []<std::size_t... I>(std::index_sequence<I...>)
	{
		return (LayoutT::template hasBlock<typename ViewLayoutT::template Block<I>::name>() && ...);
	}(std::make_index_sequence<ViewLayoutT::blockCount>{});

	static_assert(allBlocksPresent, "View layout must be a subset of storage layout");

	constexpr auto componentIndices = []<std::size_t... I>(std::index_sequence<I...>)
	{
		return typename View<ViewLayoutT, ConstAccess>::ComponentIndices{
			LayoutT::template findBlockIndex<typename ViewLayoutT::template Block<I>::name>()...
		};
	}(std::make_index_sequence<ViewLayoutT::blockCount>{});

	return View<ViewLayoutT, ConstAccess>(mStorage, mOffsets.data(), mSize, componentIndices);
}

} // namespace nph::ecs
