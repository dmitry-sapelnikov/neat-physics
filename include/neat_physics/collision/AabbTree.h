// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025-2026 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

// Includes
#include <neat_physics/collision/Aabb.h>

namespace nph
{

/// D-dimensional dynamic AABB tree for broad-phase collision detection.
/// Based on Erin Catto "Dynamic Bounding Volume Hierarchies"
/// https://box2d.org/files/ErinCatto_DynamicBVH_Full.pdf
template <uint16_t D>
class AabbTree
{
public:
	/// Constructor
	/// \param aabbExpansionFactor AABB expansion factor. Must be >= 0.
	/// The tree will store AABBs expanded by this factor to
	/// let actual AABBs move inside them without triggering tree updates.
	explicit AabbTree();

	/// Clears the tree
	void clear();

	/// Returns user data associated with a leaf node
	/// Asserts that \p leafNodeId is a leaf node
	uint32_t getUserData(uint32_t leafNodeId) const;

	/// Returns the AABB of a node
	const Aabb<D>& getAabb(uint32_t nodeId) const;

	/// Adds an AABB with user data to the tree as a leaf
	/// \return the leaf node ID associated with the AABB
	uint32_t addAabb(
		const Aabb<D>& aabb,
		float expansionFactor,
		uint32_t userData);

	/// Moves a existing AABB. Asserts that \p nodeId is a leaf node
	/// \return true if the leaf node associated with the AABB was re-inserted
	bool moveAabb(
		uint32_t nodeId,
		const Aabb<D>& newAabb,
		float expansionFactor);

	/// Tests the overlap of the provided AABB with the nodes in the tree.
	/// Calls the callback for each overlapping leaf node
	/// The callback has the signature: void callback(uint32_t nodeId);
	template <typename CallbackType>
	void testOverlap(const Aabb<D>& aabb, CallbackType&& callback) const;

	/// Process all AABB nodes in the tree
	/// Calls the callback for each node in the tree
	/// The callback has the signature: void callback(const Aabb<D>& aabb, int32_t height);
	template <typename CallbackType>
	void processAllNodes(CallbackType&& callback) const;

private:
	/// Null node flag value
	static constexpr uint32_t NULL_NODE = std::numeric_limits<uint32_t>::max();

	/// Null height value for free nodes in the pool
	static constexpr int32_t NULL_HEIGHT = -1;

	/// Initial capacity of the node pool
	static constexpr size_t INITIAL_NODE_CAPACITY = 16;

	/// Initial capacity of the stack
	static constexpr size_t INITIAL_STACK_CAPACITY = 256;

	/// Processing stack type
	using StackType = std::vector<uint32_t>;

	/// Tree node structure
	struct Node
	{
		/// AABB
		Aabb<D> aabb;

		/// User data. Meaningful only for leaf nodes.
		uint32_t userData;

		/// Parent node index
		uint32_t parent;

		/// Child #1
		uint32_t child1;

		/// Child #2
		uint32_t child2;

		// Node height, leaf = 0, free node = NULL_HEIGHT
		int32_t height;

		/// Checks if the node is a leaf node
		bool isLeaf() const
		{
			assert(height != 0 || (child1 == NULL_NODE && child2 == NULL_NODE));
			return height == 0;
		}
	};

	/// Builds a linked list for the free nodes as
	/// a vertical tree (i.e. a linked list)
	/// starting from \p startNode index till the end of the node pool.
	/// The next free node index is stored in the 'parent' field of the node.
	/// \param startNodeId The node pool will be turned
	/// into a free node list starting from this index
	void createFreeNodeList(uint32_t startNodeId);

	/// Allocates a node from the pool. Expands the node pool if necessary.
	/// The result node has:
	/// - 0 for its user data
	/// - NULL_NODE for its parent and children
	/// - uninitialized AABB
	uint32_t allocateNode();

	/// Frees a node, returns it to the pool.
	void freeNode(uint32_t nodeId);

	/// Updates the tree starting from the given node index up to the root.
	void updateToRoot(uint32_t nodeId);

	/// Finds the best sibling for a leaf node for insertion.
	uint32_t findBestSibling(uint32_t leafNodeId) const;

	/// Creates a new parent node for an existing sibling node
	/// and a new leaf node.
	/// \param siblingId Index of an existing 'sibling' node in the tree
	/// \param leafId Index of the new leaf node to be added
	void createParentForSiblingAndLeaf(
		uint32_t siblingId,
		uint32_t leafId);

	/// Adds a leaf node into the tree.
	/// Asserts that \p node is a leaf node.
	void addLeaf(uint32_t node);

	/// Removes a leaf node from the tree.
	/// Asserts that \p node is a leaf node.
	void removeLeaf(uint32_t node);

	/// Balances the tree at the given node and returns the new root index.
	uint32_t balance(uint32_t index);

	/// Index of the root node of the tree
	uint32_t mRootId;

	/// Aabb nodes pool
	std::vector<Node> mNodes;

	/// Number of currently allocated nodes in the pool
	uint32_t mNodesCount;

	/// Index of the first node in the free list
	uint32_t mFreeNodeList;
};

template <uint16_t D>
AabbTree<D>::AabbTree()
{
	mNodes.resize(INITIAL_NODE_CAPACITY);
	clear();
}

template <uint16_t D>
void AabbTree<D>::clear()
{
	mRootId = NULL_NODE;
	mNodesCount = 0;
	createFreeNodeList(0);
}

template <uint16_t D>
uint32_t AabbTree<D>::getUserData(uint32_t leafNodeId) const
{
	const Node& result = mNodes[leafNodeId];
	assert(result.isLeaf());
	return result.userData;
}

template <uint16_t D>
const Aabb<D>& AabbTree<D>::getAabb(uint32_t nodeId) const
{
	return mNodes[nodeId].aabb;
}

template <uint16_t D>
uint32_t AabbTree<D>::addAabb(
	const Aabb<D>& aabb,
	float expansionFactor,
	uint32_t userData)
{
	assert(expansionFactor >= 0.0f);

	const uint32_t nodeId = allocateNode();
	Node& node = mNodes[nodeId];
	node.aabb = aabb.getExpanded(expansionFactor);
	node.userData = userData;

	addLeaf(nodeId);
	return nodeId;
}

template <uint16_t D>
bool AabbTree<D>::moveAabb(
	uint32_t nodeId,
	const Aabb<D>& newAabb,
	float expansionFactor)
{
	assert(expansionFactor >= 0.0f);

	Node& node = mNodes[nodeId];
	assert(node.isLeaf());

	if (node.aabb.contains(newAabb))
	{
		/// \todo: think on usage of the 'huge AABB optimization' from Box2D
		return false;
	}

	removeLeaf(nodeId);
	node.aabb = newAabb.getExpanded(expansionFactor);
	addLeaf(nodeId);

	return true;
}

template <uint16_t D>
template <typename CallbackType>
inline void AabbTree<D>::testOverlap(
	const Aabb<D>& aabb,
	CallbackType&& callback) const
{
	//	Thread local storage stack
	static thread_local StackType stack;

	if (mRootId == NULL_NODE)
	{
		return;
	}

	stack.clear();
	stack.reserve(INITIAL_STACK_CAPACITY);
	stack.push_back(mRootId);

	while (!stack.empty())
	{
		const uint32_t nodeId = stack.back();
		stack.pop_back();

		const Node& node = mNodes[nodeId];
		if (node.aabb.overlaps(aabb))
		{
			if (node.isLeaf())
			{
				callback(nodeId);
			}
			else
			{
				if (node.child1 != NULL_NODE)
				{
					stack.push_back(node.child1);
				}

				if (node.child2 != NULL_NODE)
				{
					stack.push_back(node.child2);
				}
			}
		}
	}
}

template <uint16_t D>
template <typename CallbackType>
void AabbTree<D>::processAllNodes(CallbackType&& callback) const
{
	for (const auto& node : mNodes)
	{
		if (node.height >= 0)
		{
			callback(node.aabb, node.height);
		}
	}
}

template <uint16_t D>
void AabbTree<D>::createFreeNodeList(uint32_t startNodeId)
{
	for (uint32_t i = startNodeId; i < mNodes.size(); ++i)
	{
		mNodes[i].parent = i + 1;
		mNodes[i].height = NULL_HEIGHT;
	}
	mNodes.back().parent = NULL_NODE;
	mFreeNodeList = startNodeId;
}

template <uint16_t D>
uint32_t AabbTree<D>::allocateNode()
{
	// Expand the node pool if needed.
	if (mFreeNodeList == NULL_NODE)
	{
		assert(mNodesCount == mNodes.size());
		mNodes.resize(2 * mNodes.size());
		createFreeNodeList(mNodesCount);
	}

	const uint32_t nodeId = mFreeNodeList;
	Node& node = mNodes[mFreeNodeList];
	// Move the free node list head to the next free node
	mFreeNodeList = node.parent;

	node.userData = 0;
	node.parent = NULL_NODE;
	node.child1 = NULL_NODE;
	node.child2 = NULL_NODE;
	node.height = 0;

	++mNodesCount;
	return nodeId;
}

template <uint16_t D>
void AabbTree<D>::freeNode(uint32_t nodeId)
{
	assert(mNodesCount > 0);
	Node& node = mNodes[nodeId];
	assert(node.height >= 0);

	// Move the freed node to the head of the free node list
	node.parent = mFreeNodeList;
	node.height = NULL_HEIGHT;
	mFreeNodeList = nodeId;
	--mNodesCount;
}

template <uint16_t D>
uint32_t AabbTree<D>::findBestSibling(uint32_t leafNodeId) const
{
	assert(mRootId != NULL_NODE);

	const Node& rootNode = mNodes[mRootId];
	if (rootNode.isLeaf())
	{
		return mRootId;
	}

	const Aabb<D>& leafAabb = mNodes[leafNodeId].aabb;
	const Vec<D> leafCenter = leafAabb.getCenter();
	const float leafArea = getSurfaceArea(leafAabb);

	float nodeArea = getSurfaceArea(rootNode.aabb);
	float directCost = getSurfaceArea(rootNode.aabb + leafAabb);
	float inheritedCost = 0.0f;

	uint32_t bestSiblingId = mRootId;
	float bestCost = directCost;

	/// A helper lambda to process a child node
	auto processChild = [&](
		const Node& childNode,
		uint32_t childId,
		float& directCost,
		float& possibleCost,
		float& childArea)
		{
			directCost = getSurfaceArea(childNode.aabb + leafAabb);
			if (childNode.isLeaf())
			{
				// Since we don't choose leaf nodes as next for descend,
				// we need to check their cost right here
				if (const float cost = directCost + inheritedCost;
					cost < bestCost)
				{
					bestSiblingId = childId;
					bestCost = cost;
				}
			}
			else
			{
				// We compute the child area only for non-leaf nodes
				childArea = getSurfaceArea(childNode.aabb);

				// There are 2 options for a non-leaf child:
				// 1. The child could be the sibling, in which case the cost is
				// inheritedCost + directCost
				// 2. A descendent of the child could be the sibling,
				// in which case the cost is
				// inheritedCost + (directCost - childArea) + leafArea
				// We choose the minimum of these 2 values
				possibleCost =
					inheritedCost + directCost +
					std::min(leafArea - childArea, 0.0f);
			}
		};

	uint32_t nodeId = mRootId;
	// Do a greedy descent of the tree starting from the root node
	for (;;)
	{
		if (const float cost = directCost + inheritedCost;
			cost < bestCost)
		{
			bestSiblingId = nodeId;
			bestCost = cost;
		}

		inheritedCost += (directCost - nodeArea);

		const uint32_t childId1 = mNodes[nodeId].child1;
		const uint32_t childId2 = mNodes[nodeId].child2;

		const Node& childNode1 = mNodes[childId1];
		const Node& childNode2 = mNodes[childId2];

		float possibleCost1 = FLT_MAX;
		float area1 = 0.0f;
		float directCost1;
		processChild(childNode1, childId1, directCost1, possibleCost1, area1);

		float possibleCost2 = FLT_MAX;
		float area2 = 0.0f;
		float directCost2;
		processChild(childNode2, childId2, directCost2, possibleCost2, area2);

		if (bestCost <= possibleCost1 &&
			bestCost <= possibleCost2)
		{
			break;
		}

		if (possibleCost1 == possibleCost2)
		{
			// In this tie case let's use center distance as the cost function
			possibleCost1 = (childNode1.aabb.getCenter() - leafCenter).lengthSquared();
			possibleCost2 = (childNode2.aabb.getCenter() - leafCenter).lengthSquared();
		}

		if (possibleCost1 < possibleCost2)
		{
			nodeId = childId1;
			nodeArea = area1;
			directCost = directCost1;
		}
		else
		{
			nodeId = childId2;
			nodeArea = area2;
			directCost = directCost2;
		}
	}
	return bestSiblingId;
}

template <uint16_t D>
void AabbTree<D>::updateToRoot(uint32_t nodeId)
{
	while (nodeId != NULL_NODE)
	{
		nodeId = balance(nodeId);
		Node& node = mNodes[nodeId];
		Node& childNode1 = mNodes[node.child1];
		Node& childNode2 = mNodes[node.child2];

		node.aabb = childNode1.aabb + childNode2.aabb;
		node.height = 1 + std::max(childNode1.height, childNode2.height);

		nodeId = node.parent;
	}
}

template <uint16_t D>
void AabbTree<D>::createParentForSiblingAndLeaf(
	uint32_t siblingId,
	uint32_t leafId)
{
	assert(mNodes[leafId].isLeaf());

	const uint32_t newParentId = allocateNode();
	Node& newParent = mNodes[newParentId];

	// We need to create these references after createNode() call
	// because createNode() may cause invalidation of references
	// due to reallocation
	Node& oldNode = mNodes[siblingId];
	Node& newNode = mNodes[leafId];
	const uint32_t oldParent = oldNode.parent;

	newParent.parent = oldParent;
	newParent.userData = 0;
	newParent.aabb = newNode.aabb + oldNode.aabb;
	newParent.height = oldNode.height + 1;

	newParent.child1 = siblingId;
	newParent.child2 = leafId;
	oldNode.parent = newParentId;
	newNode.parent = newParentId;

	if (oldParent != NULL_NODE)
	{
		Node& oldParentNode = mNodes[oldParent];
		if (oldParentNode.child1 == siblingId)
		{
			oldParentNode.child1 = newParentId;
		}
		else
		{
			assert(oldParentNode.child2 == siblingId);
			oldParentNode.child2 = newParentId;
		}
	}
	else
	{
		mRootId = newParentId;
	}
	updateToRoot(newParentId);
}

template <uint16_t D>
void AabbTree<D>::addLeaf(uint32_t leafId)
{
	assert(mNodes[leafId].isLeaf());
	if (mRootId == NULL_NODE)
	{
		mRootId = leafId;
		mNodes[leafId].parent = NULL_NODE;
		return;
	}
	createParentForSiblingAndLeaf(findBestSibling(leafId), leafId);
}

template <uint16_t D>
void AabbTree<D>::removeLeaf(uint32_t leafId)
{
	assert(mNodes[leafId].isLeaf());

	if (leafId == mRootId)
	{
		mRootId = NULL_NODE;
		return;
	}

	Node& leaf = mNodes[leafId];
	const uint32_t parentId = leaf.parent;
	Node& parent = mNodes[parentId];

	const uint32_t siblingId = (parent.child1 == leafId) ?
		parent.child2 :
		parent.child1;
	Node& siblingNode = mNodes[siblingId];

	if (uint32_t grandParentId = parent.parent;
		grandParentId != NULL_NODE)
	{
		Node& grandParent = mNodes[grandParentId];
		// Destroy parent and connect sibling to grandParent.
		if (grandParent.child1 == parentId)
		{
			grandParent.child1 = siblingId;
		}
		else
		{
			assert(grandParent.child2 == parentId);
			grandParent.child2 = siblingId;
		}
		siblingNode.parent = grandParentId;
		freeNode(parentId);
		updateToRoot(grandParentId);
	}
	else
	{
		mRootId = siblingId;
		mNodes[siblingId].parent = NULL_NODE;
		freeNode(parentId);
	}
}

template <uint16_t _D>
uint32_t AabbTree<_D>::balance(uint32_t nodeId)
{
	Node& nodeA = mNodes[nodeId];
	if (nodeA.isLeaf() || nodeA.height < 2)
	{
		return nodeId;
	}

	const int32_t balance =
		mNodes[nodeA.child2].height -
		mNodes[nodeA.child1].height;

	if (-2 < balance && balance < 2)
	{
		return nodeId;
	}

	// Here we have a tree with implance between
	// its subtrees > 1, so we need to perform tree rotation to balance it
	// The node configuration before balancing is:
	//
	//     A
	//    / \
	//   B   C
	//      / \
	//     D   E
	//    / \
	//   F   G
	//
	// We assume that the C node has bigger height than B,
	// and D has bigger height than E.
	// If it is not so, we swap B and C and then D and E in our computation.
	// For the depicted configuration, the result after 
	// the tree rotation will be:
	//
	//      C
	//     / \
	//    A   D
	//   / \ / \
	//  B  E F  G

	uint32_t* ptrIdB = &nodeA.child1;
	uint32_t* ptrIdC = &nodeA.child2;
	// We assume that C has bigger height than B
	// If not, we swap them
	if (balance < 0)
	{
		std::swap(ptrIdB, ptrIdC);
	}

	Node& nodeB = mNodes[*ptrIdB];
	const uint32_t idC = *ptrIdC;
	Node& nodeC = mNodes[idC];

	nodeC.parent = nodeA.parent;
	nodeA.parent = idC;
	if (nodeC.parent != NULL_NODE)
	{
		Node& cParent = mNodes[nodeC.parent];
		if (cParent.child1 == nodeId)
		{
			cParent.child1 = idC;
		}
		else
		{
			assert(cParent.child2 == nodeId);
			cParent.child2 = idC;
		}
	}
	else
	{
		mRootId = idC;
	}

	uint32_t idD = nodeC.child1;
	uint32_t idE = nodeC.child2;
	// We assume that D has bigger height than E, if not we swap them
	if (mNodes[idD].height <= mNodes[idE].height)
	{
		std::swap(idD, idE);
	}

	nodeC.child1 = nodeId;
	nodeC.child2 = idD;

	*ptrIdC = idE;

	Node& nodeD = mNodes[idD];
	Node& nodeE = mNodes[idE];

	nodeE.parent = nodeId;

	// Update of A should go first
	nodeA.aabb = nodeB.aabb + nodeE.aabb;
	nodeA.height = 1 + std::max(nodeB.height, nodeE.height);

	// Then update of C
	nodeC.aabb = nodeA.aabb + nodeD.aabb;
	nodeC.height = 1 + std::max(nodeA.height, nodeD.height);
	return idC;
}

} // namespace nph
