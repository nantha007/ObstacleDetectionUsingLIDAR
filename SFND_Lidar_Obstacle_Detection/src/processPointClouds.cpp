// PCL lib Functions for processing point clouds

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    pcl::VoxelGrid<PointT> vg;
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>);
    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes, filterRes, filterRes);
    vg.filter(*cloudFiltered);

    typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>);

    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);

    std::vector<int> indices;

    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f (2.6, 1.7, -0.4, 1));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for(int point:indices)
        inliers->indices.push_back(point);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudRegion);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr roadCloud (new pcl::PointCloud<PointT> ());

    for (int index: inliers-> indices)
        roadCloud->points.push_back(cloud->points[index]);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstCloud);


    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, roadCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	// pcl::PointIndices::Ptr inliersResult;
  //  TODO:: Fill in this function to find inliers for the cloud.
    std::unordered_set<int> inliersResult;
	srand(time(NULL));

	// TODO: Fill in this function

	// For max iterations
	for(int i = 0; i < maxIterations; i++){
		// Randomly sample subset and fit line
        // pcl::PointIndices::Ptr inliers;
		std::unordered_set<int> inliers;
		while(inliers.size() < 3)
			inliers.insert(rand()%(cloud->points.size()));
		float x1, y1, x2, y2, x3, y3, z1, z2, z3;
		auto itr = inliers.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;
		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z;
		itr++;
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
		z3 = cloud->points[*itr].z;

		float A, B, C, D;
		A = (y1-y2)*(z3-z1) - (z2-z1)*(y3-y1);
		B = (z2-z1)*(x3-x1) - (x2-x1)*(z3-z1);
		C = (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1);
		D = -(A*x1 + B*y1 + C*z1);
		float dist;
		for (int idx = 0; idx < cloud->points.size(); idx++){
			if (inliers.count(idx)>0)
				continue;
			auto point = cloud->points[idx];
			float x3, y3, z3;
			x3 = point.x;
			y3 = point.y;
			z3 = point.z;

			dist = fabs(A*x3 + B*y3 + C*z3 + D)/sqrt(A*A + B*B + C*C);

			if (dist<distanceThreshold ){
				inliers.insert(idx);
			}
		}
		if (inliers.size() > inliersResult.size()){
			inliersResult = inliers;
		}

		// Measure distance between every point and fitted line
		// If distance is smaller than threshold count it as inlier
  }

  // pcl::SACSegmentation<PointT> seg;
  pcl::PointIndices::Ptr inliers_cloud {new pcl::PointIndices};
  for (int point:inliersResult)
    inliers_cloud->indices.push_back(point);
  // pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients};
  //
  // seg.setOptimizeCoefficients(true);
  // seg.setModelType(pcl::SACMODEL_PLANE);
  // seg.setMethodType(pcl::SAC_RANSAC);
  // seg.setMaxIterations(maxIterations);
  // seg.setDistanceThreshold(distanceThreshold);
  //
  // //segment
  //
  // seg.setInputCloud(cloud);
  // seg.segment (*inliers, *coefficients);
  // if (inliers->indices.size() == 0){
  //     std::cout << "Could not estimate a planar model for the given datasets."
  //         << std::endl;
  // }
  //


  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers_cloud,cloud);
  return segResult;
}

template<typename PointT>
void ProcessPointClouds<PointT>::proximity(int point_id, std::vector<bool>& processed, typename pcl::PointCloud<PointT>::Ptr cloud,
  std::vector<int>& cluster, KdTree<PointT>* tree, float distanceTol){
	  processed[point_id] = true;
	  cluster.push_back(point_id);

	  std::vector<int> ids = tree->search(cloud->points[point_id], distanceTol);

	  for(auto id:ids){
		  if(!processed[id])
		  	proximity(id, processed, cloud, cluster, tree, distanceTol);
	  }
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float distanceTol, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    // typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    // tree->setInputCloud(cloud);
    //
    // std::vector<pcl::PointIndices> clusterIndices;
    // pcl::EuclideanClusterExtraction<PointT> ec;
    // ec.setClusterTolerance(clusterTolerance);
    // ec.setMinClusterSize(minSize);
    // ec.setMaxClusterSize(maxSize);
    // ec.setSearchMethod(tree);
    // ec.setInputCloud(cloud);
    // ec.extract(clusterIndices);
    //
    // for(pcl::PointIndices getIndices:clusterIndices){
    //     typename pcl::PointCloud<PointT>::Ptr cloudCluster {new pcl::PointCloud<PointT>};
    //     for (int index : getIndices.indices)
    //         cloudCluster->points.push_back(cloud->points[index]);
    //
    //     cloudCluster->width = cloudCluster->points.size();
    //     cloudCluster->height = 1;
    //     cloudCluster->is_dense = true;
    //     clusters.push_back(cloudCluster);
    // }
    KdTree<PointT>* tree = new KdTree<PointT>;
    for (int i=0; i<cloud->points.size(); i++)
      tree->insert(cloud->points[i],i);
  	std::vector<bool> processed(cloud->points.size(), false);
  	for(int i = 0; i<cloud->points.size(); i++){
  		if (processed[i])
  			continue;
  		std::vector<int> cluster;
  		proximity(i, processed, cloud, cluster, tree, distanceTol);
      typename pcl::PointCloud<PointT>::Ptr clusterCloud {new pcl::PointCloud<PointT>()};
      for(int indice: cluster)
        clusterCloud->points.push_back(cloud->points[indice]);

      clusterCloud->width = clusterCloud->points.size();
      clusterCloud->height = 1;
      clusterCloud->is_dense = true;
  		clusters.push_back(clusterCloud);

  	}


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}
