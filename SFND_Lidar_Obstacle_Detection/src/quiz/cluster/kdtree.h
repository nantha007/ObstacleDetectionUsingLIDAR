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
	void insertHelper(Node** node, std::vector<float> point, int id, uint depth){
		uint check = depth%2;
		if(*node == NULL){
			*node = new Node(point, id);
		}
		else{
			if (point[check] < (*node)->point[check]){
				insertHelper(&(*node)->left, point, id, depth+1);
			}
			else{
				insertHelper(&(*node)->right, point, id, depth+1);
			}
		}	
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelper(&root, point, id, 0);
	}

	void searchHelper(Node* node, std::vector<float> target, float distanceTol, std::vector<int>& ids, uint depth){
		if(node!=NULL){
			if(node->point[0] >= (target[0] - distanceTol) && node->point[0] <= (target[0] + distanceTol) && 
				node->point[1] <= (target[1] + distanceTol) && node->point[1] >= (target[1] - distanceTol)){
					float dist = sqrt((node->point[0] - target[0])*(node->point[0] - target[0]) + 
						(node->point[1] - target[1])*(node->point[1] - target[1]));
					if(dist <= distanceTol){
						ids.push_back(node->id);
					}
			}
			if ( (target[depth%2]-distanceTol) < node->point[depth%2])
				searchHelper(node->left, target, distanceTol, ids, depth+1);
			if ( (target[depth%2]+distanceTol) > node->point[depth%2])
				searchHelper(node->right, target, distanceTol, ids, depth+1);
		}
	}
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(root, target, distanceTol, ids, 0);
		return ids;
	}
	

};




