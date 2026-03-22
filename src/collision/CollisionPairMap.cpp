// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025-2026 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

// Includes
#include <neat_physics/collision/CollisionPairMap.h>

namespace nph
{

uint32_t* CollisionPairMap::at(uint32_t bodyA, uint32_t bodyB) noexcept
{
	const uint64_t key = makeKey(bodyA, bodyB);

	Entry& entry = mEntries[findEntryIndex(key)];
	assert(entry.key == 0 || entry.key == key);

	return entry.key != 0 ? &entry.value : nullptr;
}

void CollisionPairMap::startUpdate(const BodyMoveFlags& moveFlags) noexcept
{
	for (Entry& entry : mEntries)
	{
		entry.obsolete =
			entry.key != 0 &&
			(moveFlags[entry.key >> 32] || moveFlags[entry.key & 0xffffffff]);
	}
}

void CollisionPairMap::addPair(uint32_t bodyA, uint32_t bodyB)
{
	const uint64_t key = makeKey(bodyA, bodyB);
	uint32_t ind = findEntryIndex(key);
	Entry& result = mEntries[ind];

	// Already in the map, just mark as non-obsolete
	if (result.key != 0)
	{
		assert(result.key == key);
		result.obsolete = false;
		return;
	}

	// Keep load factor <= 0.5 to maintain performance of the hash table
	if (mSize * 2 >= mEntries.size())
	{
		doubleEntries();
	}

	Entry& newEntry = mEntries[addKey(key)];
	// Since addKey initializes only the key field,
	// we need to initialize the rest of the entry here.
	newEntry.value = NULL_VALUE;
	newEntry.obsolete = false;
}

uint32_t CollisionPairMap::findEntryIndex(uint64_t key) const noexcept
{
	const uint32_t capacityMask =
		static_cast<uint32_t>(mEntries.size()) - 1;

	uint32_t index =
		static_cast<uint32_t>(getKeyHash(key)) & capacityMask;

	while (
		mEntries[index].key != 0 &&
		mEntries[index].key != key)
	{
		index = (index + 1) & capacityMask;
	}

	return index;
}

uint32_t CollisionPairMap::addKey(uint64_t key) noexcept
{
	const uint32_t ind = findEntryIndex(key);
	Entry& entry = mEntries[ind];
	assert(entry.key == 0);
	entry.key = key;
	++mSize;
	return ind;
}

void CollisionPairMap::doubleEntries()
{
	mSize = 0;
	std::vector<Entry> oldEntries(mEntries.size() * 2);
	std::swap(mEntries, oldEntries);
	resetMemory();

	for (const Entry& oldEntry : oldEntries)
	{
		if (oldEntry.key != 0)
		{
			mEntries[addKey(oldEntry.key)] = oldEntry;
		}
	}
}

void CollisionPairMap::removeKey(uint64_t key) noexcept
{
	uint32_t removedInd = findEntryIndex(key);
	Entry& entryToRemove = mEntries[removedInd];
	assert(entryToRemove.key != 0);
	entryToRemove.key = 0;

	assert(mSize > 0);
	--mSize;

	const uint32_t capacityMask =
		static_cast<uint32_t>(mEntries.size()) - 1;

	/// The Robin Hood hashing strategy:
	/// Restore continuity of the hash clusters by
	/// rehashing subsequent keys until we find an empty slot
	uint32_t testInd = removedInd;
	for (;;)
	{
		testInd = (testInd + 1) & capacityMask;
		Entry& probeEntry = mEntries[testInd];

		// We discovered the boundary between 2 hash clusters,
		// no more items to rehash
		if (probeEntry.key == 0)
		{
			break;
		}

		// The first item in the hash cluster of probeInd entry
		const uint32_t firstInd =
			static_cast<uint32_t>(getKeyHash(probeEntry.key)) & capacityMask;

		// check if f lies in (r, t] interval, where
		// treating the array as circular:
		// r <= t: [r f t]
		// r > t:  [f t r] or [t r f]
		if (removedInd <= testInd)
		{
			if (removedInd < firstInd && firstInd <= testInd)
			{
				continue;
			}
		}
		else
		{
			if (firstInd <= testInd || firstInd > removedInd)
			{
				continue;
			}
		}

		// If the firstInd does not lie in the interval,
		// we need to move the firstInd entry to the removedInd slot
		mEntries[removedInd] = mEntries[testInd];
		mEntries[testInd].key = 0;
		removedInd = testInd;
	}
}

} // namespace nph
