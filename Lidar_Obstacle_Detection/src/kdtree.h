/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insertHelper(Node * &node, uint depth, std::vector<float> point, int id)
	{
		if (node == NULL)
		{
			node = new Node(point, id);
		}
		else
		{
			uint dim = depth % 3; // 3D

			if (point[dim] < node->point[dim])
			{
				insertHelper(node->left, depth + 1, point, id);
			}
			else
			{
				insertHelper(node->right, depth + 1, point, id);
			}
		}
	}

	void insert(std::vector<float> point, int id)
	{
		insertHelper(root, 0, point, id);
	}

	void searchHelper(std::vector<float> target, Node * node, int depth, float distanceTol, std::vector<int>& ids)
	{
		if (node != NULL)
		{
			// Point is within the targets match tolerance box (3D)
			if ((node->point[0] >= (target[0] - distanceTol)) &&
			    (node->point[0] <= (target[0] + distanceTol)) &&
				(node->point[1] >= (target[1] - distanceTol)) &&
				(node->point[1] <= (target[1] + distanceTol)) &&
				(node->point[2] >= (target[2] - distanceTol)) &&
				(node->point[2] <= (target[2] + distanceTol)))
			{
				// 3D distance calculation
				float distance = sqrt(pow(node->point[0] - target[0], 2) + pow(node->point[1] - target[1], 2) + pow(node->point[2] - target[2], 2));

				if (distance <= distanceTol)
				{
					ids.push_back(node->id);
				}
			}

			if ((target[depth % 3] - distanceTol) < node->point[depth % 3]) // 3D
			{
				searchHelper(target, node->left, depth + 1, distanceTol, ids);
			}

			if ((target[depth % 3] + distanceTol) > node->point[depth % 3]) // 3D
			{
				searchHelper(target, node->right, depth + 1, distanceTol, ids);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;

		searchHelper(target, root, 0, distanceTol, ids);

		return ids;
	}
};
