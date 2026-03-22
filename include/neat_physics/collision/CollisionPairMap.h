// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025-2026 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

// Includes
#include <cassert>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>

namespace nph
{

/// Map for storing collision pairs.
/// The collision pair is a pair of bodies which extended AABBs
/// in the dynamic tree intersect.
/// Map values are indices of contact manifolds.
/// The map uses open-addressing hash table with
/// linear probing for collision resolution.
class CollisionPairMap
{
public:
	/// Sentinel value for unoccupied values
	static constexpr uint32_t NULL_VALUE = std::numeric_limits<uint32_t>::max();

	/// Map default capacity
	static constexpr uint32_t DEFAULT_CAPACITY = 16;

	/// Flags indicating whether a body has moved during the current step
	using BodyMoveFlags = std::vector<uint8_t>;

	/// Constructor. Initializes the map with a default capacity.
	CollisionPairMap()
	{
		mEntries.resize(DEFAULT_CAPACITY);
		resetMemory();
	}

	/// Clears the map
	void clear() noexcept
	{
		mSize = 0;
		resetMemory();
	}

	/// Gets the value for the pair of bodies, 
	/// or nullptr if the pair is not in the map.
	[[nodiscard]] uint32_t* at(uint32_t bodyA, uint32_t bodyB) noexcept;

	/// Starts the update of the map for the current step.
	/// Marks pairs involving moved bodies as obsolete.
	void startUpdate(const BodyMoveFlags& moveFlags) noexcept;

	/// Ends the update of the map for the current step:
	/// - removes pairs marked as obsolete
	/// - calls the callback for non-obsolete pairs
	template <typename CollisionCallback>
	void endUpdate(CollisionCallback&& collisionCallback);

	/// Adds the pair of bodies to the map if not already present
	/// Sets the pair as non-obsolete
	/// Asserts that bodyA < bodyB.
	void addPair(uint32_t bodyA, uint32_t bodyB);

private:
	/// An entry in the map
	/// 16 bytes in size for better cache performance
	/// The value is a uint32_t index of the contact manifold
	struct Entry
	{
		/// Key is a pair of body indices (bodyA << 32 | bodyB)
		uint64_t key;

		/// Value is the index of the contact manifold, 
		/// or NULL_VALUE if unoccupied
		uint32_t value;

		/// Obsolete flag used to prune obsolete pairs during the update
		/// Intentionally made uint32_t for 16-byte padding.
		uint32_t obsolete;
	};
	static_assert(sizeof(Entry) == 16, "Entry should be 16 bytes in size");

	/// Vector of entries in the map
	using Entries = std::vector<Entry>;

	/// Converts a pair of body indices to a key for the map
	static [[nodiscard]] uint64_t makeKey(uint32_t bodyA, uint32_t bodyB) noexcept
	{
		assert(bodyA < bodyB);
		return(static_cast<uint64_t>(bodyA) << 32) | static_cast<uint64_t>(bodyB);
	}

	/// Hashes the key using MurmurHash3 algorithm.
	/// Asserts that key is not zero,
	/// as zero is used to indicate unoccupied slots.
	static [[nodiscard]] uint64_t getKeyHash(uint64_t key) noexcept
	{
		assert(key != 0);
		uint64_t h = key;
		h ^= h >> 33;
		h *= 0xff51afd7ed558ccduLL;
		h ^= h >> 33;
		h *= 0xc4ceb9fe1a85ec53uLL;
		h ^= h >> 33;
		return h;
	}

	/// Finds the entry for the given key
	[[nodiscard]] uint32_t findEntryIndex(uint64_t key) const noexcept;

	/// Adds the key to the map at the slot determined by the hash.
	/// \note Does NOT initialize the entry other than setting the key.
	[[nodiscard]] uint32_t addKey(uint64_t key) noexcept;

	/// Doubles the table of entries and rehashes all the existing keys
	void doubleEntries();

	/// Removes the key from the map and
	/// fills the gap by rehashing subsequent keys.
	void removeKey(uint64_t key) noexcept;

	/// Resets the memory of the entries vector to zero.
	void resetMemory() noexcept
	{
		// \todo check if this is safe
		std::memset(mEntries.data(), 0, mEntries.size() * sizeof(Entry));
	}

	/// Entries in the map. 
	/// Key is a pair of body indices (bodyA << 32 | bodyB),
	/// value is the index of the contact manifold.
	/// The size is always a power of two 
	/// to simplify the hash function and collision resolution.
	Entries mEntries;

	/// Number of pairs in the map
	uint32_t mSize = 0;
};

template <typename CollisionCallback>
void CollisionPairMap::endUpdate(CollisionCallback&& collisionCallback)
{
	const size_t capacity = mEntries.size();
	size_t ind = 0;
	while (ind < capacity)
	{
		Entry& entry = mEntries[ind];
		if (entry.key == 0)
		{
			++ind;
			continue;
		}

		if (entry.obsolete)
		{
			removeKey(entry.key);
		}
		else
		{
			collisionCallback(
				static_cast<uint32_t>(entry.key >> 32),
				static_cast<uint32_t>(entry.key & 0xffffffff),
				entry.value);
			++ind;
		}
	}
}

} // namespace nph
