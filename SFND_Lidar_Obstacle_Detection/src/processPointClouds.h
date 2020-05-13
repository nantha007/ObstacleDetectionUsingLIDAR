// PCL lib Functions for processing point clouds

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream>
#include <string>
#include <vector>
#include <ctime>
#include <chrono>
#include <unordered_set>
#include "render/box.h"

// #include "kdtree.h"


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

template<typename PointT>
struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}
	void insertHelper(Node** node, PointT point, int id, uint depth){
		uint check = depth%3;
		if(*node == NULL){
			std::vector<float> vec{point.x, point.y, point.z};
			*node = new Node(vec, id);
		}
		else{
			if (point.data[check] < (*node)->point[check]){
				insertHelper(&(*node)->left, point, id, depth+1);
			}
			else{
				insertHelper(&(*node)->right, point, id, depth+1);
			}
		}
	}

	void insert(PointT point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		insertHelper(&root, point, id, 0);
	}

	void searchHelper(Node* node, PointT target, float distanceTol, std::vector<int>& ids, uint depth){
		if(node!=NULL){
			if(node->point[0] >= (target.data[0] - distanceTol) && node->point[0] <= (target.data[0] + distanceTol) &&
				node->point[1] <= (target.data[1] + distanceTol) && node->point[1] >= (target.data[1] - distanceTol) &&
        node->point[2] <= (target.data[2] + distanceTol) && node->point[2] >= (target.data[2] - distanceTol)){
					float dist = sqrt((node->point[0] - target.data[0])*(node->point[0] - target.data[0]) +
						(node->point[1] - target.data[1])*(node->point[1] - target.data[1]) +
            (node->point[2] - target.data[2])*(node->point[2] - target.data[2]));
					if(dist <= distanceTol){
						ids.push_back(node->id);
					}
			}
			if ( (target.data[depth%3]-distanceTol) < node->point[depth%3])
				searchHelper(node->left, target, distanceTol, ids, depth+1);
			if ( (target.data[depth%3]+distanceTol) > node->point[depth%3])
				searchHelper(node->right, target, distanceTol, ids, depth+1);
		}
	}
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(PointT target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(root, target, distanceTol, ids, 0);
		return ids;
	}


};

template<typename PointT>
class ProcessPointClouds {
public:

    //constructor
    ProcessPointClouds();
    //deconstructor
    ~ProcessPointClouds();

    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

    typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

    // proximity
		void proximity(int point_id, std::vector<bool>& processed, typename pcl::PointCloud<PointT>::Ptr cloud,
		  std::vector<int>& cluster, KdTree<PointT>* tree, float distanceTol);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);

};
#endif /* PROCESSPOINTCLOUDS_H_ */
