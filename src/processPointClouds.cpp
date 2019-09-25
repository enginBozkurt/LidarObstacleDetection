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
/*FilterCloud function filters the given cloud. Following operations are performed
 * Downsampling: points are converted to voxels using the dimensions provided.
 * Crop: Remove all the points that are outside the min , max limits
 * RoofCrop: Remove roof points , dimensions of roof are given.
 * */

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>());

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

    // Convert the points to voxel grid points
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (filterRes, filterRes, filterRes);
    sor.filter (*cloud_filtered);
    std::cerr << "Voxeled " << cloud_filtered->points.size () << std::endl;

    // Crop the scene to create ROI
    pcl::CropBox<PointT> roi;
    roi.setMin(minPoint);
    roi.setMax(maxPoint);
    roi.setInputCloud(cloud_filtered);
    roi.filter(*cloud_filtered);
    std::cerr << "ROI " << cloud_filtered->points.size () << std::endl;

    //Remove all the points from roof
    std::vector<int> indices;
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5,-1.7,-1,1));
    roof.setMax(Eigen::Vector4f(2.6,1.7,-.4,1));
    roof.setInputCloud(cloud_filtered);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
    for(int point:indices)
    	inliers->indices.push_back(point);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_filtered);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_filtered;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
	typename pcl::PointCloud<PointT>::Ptr obstCloud(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>());

	for(int it : inliers->indices)
	{
		planeCloud->points.push_back(cloud->points[it]);
	}

	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud (cloud);
	extract.setIndices (inliers);
	extract.setNegative (true);
	extract.filter (*obstCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
    // TODO:: Fill in this function to find inliers for the cloud.
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
	// Create the segmentation object
	pcl::SACSegmentation<PointT> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (maxIterations);
	seg.setDistanceThreshold (distanceThreshold);

	// Segment the largest planar component from the remaining cloud
	seg.setInputCloud (cloud);
	seg.segment (*inliers, *coefficients);
	if (inliers->indices.size () == 0)
	{
	  std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
	}
	std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;


    return segResult;
}
/*SegmentPlane_RANSAC function implements RANSAC function.
 * It is used for identifying the road surface.
 * Algo: Randomly choose 3 points from the cloud, form a plane using these points.
 * 		 Loop through all the points in the cloud, for each of the point calculate
 * 		 	the distance to the plane created above. If the distance is below distanceTol
 * 		 	then add the index to a temporary set
 * 		 Store the temporary vector if the size is more than previously identified indices.
 * 		 Repeat above steps for maxIterations
 * */
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane_RANSAC(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    /*Buffer to hold the indices of the points within distanceTol , it shall hold max identified indices*/
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	PointT point1;
	PointT point2;
	PointT point3;
	int idx1;
	int idx2;
	int idx3;
	float a,b,c,d,dis,len;

	// For max iterations
	for(int it=0;it<maxIterations;it++)
	{
		/*Temporary buffer to hold identified points in current loop*/
		std::unordered_set<int> tempIndices;
		/*Identify 3 points randomly*/
		while(tempIndices.size()<3)
			tempIndices.insert((rand() % cloud->points.size()));
		auto iter = tempIndices.begin();
		idx1 = *iter;
		++iter;
		idx2 = *iter;
		++iter;
		idx3 = *iter;

		point1 = cloud->points[idx1];
		point2 = cloud->points[idx2];
		point3 = cloud->points[idx3];

		/*Fit a plane using the above 3 points*/
		a = (((point2.y-point1.y)*(point3.z-point1.z))-((point2.z-point1.z)*(point3.y-point1.y)));
		b = (((point2.z-point1.z)*(point3.x-point1.x))-((point2.x-point1.x)*(point3.z-point1.z)));
		c = (((point2.x-point1.x)*(point3.y-point1.y))-((point2.y-point1.y)*(point3.x-point1.x)));
		d = -(a*point1.x+b*point1.y+c*point1.z);
		len = sqrt(a*a+b*b+c*c);

		// Measure distance between every point and fitted plane
		for(int pt_cnt=0;pt_cnt<cloud->points.size();pt_cnt++)
		{
			if(pt_cnt!=idx1||pt_cnt!=idx2||pt_cnt!=idx3)
			{
				dis = (fabs(a*cloud->points[pt_cnt].x+b*cloud->points[pt_cnt].y+c*cloud->points[pt_cnt].z+d)/len);
				// If distance is smaller than threshold count it as inlier
				if(dis<=distanceThreshold)
				{
					tempIndices.insert(pt_cnt);
				}
			}
		}

		/*Store the temporary buffer if the size if more than previously idenfitied points */
		if(tempIndices.size()>inliersResult.size())
		{
			inliersResult.clear();
			inliersResult = tempIndices;

		}

	}

	// Segment the largest planar component from the remaining cloud
	if (inliersResult.size () == 0)
	{
	  std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
	}
	/*Buffers to hold cloud and object points*/
	typename pcl::PointCloud<PointT>::Ptr cloudInliers(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

	/*Copy the points from inputcloud in to cloudInliers if the indices is in inliersResult vector
	 * or else copy the point to cloudOutliers*/
	for(int index = 0; index < cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];
		if(inliersResult.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}
	/*Create a pair using inlier and outlier points*/
	std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudOutliers, cloudInliers);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;


    return segResult;
}



template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    // Creating the KdTree object for the search method of the extraction
	typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
	tree->setInputCloud (cloud);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<PointT> ec;
	ec.setClusterTolerance (clusterTolerance); // 2cm
	ec.setMinClusterSize (minSize);
	ec.setMaxClusterSize (maxSize);
	ec.setSearchMethod (tree);
	ec.setInputCloud (cloud);
	ec.extract (cluster_indices);

	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	  {
		typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
	    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
	      cloud_cluster->points.push_back (cloud->points[*pit]); //*
	    cloud_cluster->width = cloud_cluster->points.size ();
	    cloud_cluster->height = 1;
	    cloud_cluster->is_dense = true;

	    clusters.push_back(cloud_cluster);
	  }
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

/* Proximity function shall identify all the points that are within distanceTol
 * distance from the given point in the cloud and return the indices of the points
 * This is a recurssive function.
 * Algo: If the target point is not processed then set is as processed and search
 * 			the KDTree for all the points within the distanceTol
 * 		 Use each of the nearby points and search for other points that are within
 * 		 distanceTol distance from this points
 *
 * */
template<typename PointT>
void ProcessPointClouds<PointT>::Proximity(typename pcl::PointCloud<PointT>::Ptr cloud,std::vector<int> &cluster,std::vector<bool> &processed_f,int idx,typename KdTree_euclidean<PointT>::KdTree_euclidean* tree,float distanceTol, int maxSize)
{
	if((processed_f[idx]==false)&&
			(cluster.size()<maxSize))
	{
		processed_f[idx]=true;
		cluster.push_back(idx);
		std::vector<int> nearby = tree->search(cloud->points[idx],distanceTol);
		for(int index : nearby)
		{
			if(processed_f[index]==false)
			{
				Proximity(cloud, cluster,processed_f,index,tree,distanceTol,maxSize);
			}
		}
	}

}
/* euclideanCluster function shall identify clusters that have points with in min and max limits
 * Algo: Take one point at a time from the cluster , call Proximity function to identify the
 * 			list of points that are within distanceTol limits
 * 		 Check if the no of points in cluster ,returned by proximity function, are in (minSize, maxSize)
 * 		 	limits if not discard
 * */
template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, typename KdTree_euclidean<PointT>::KdTree_euclidean* tree, float distanceTol, int minSize, int maxSize)
{
	std::vector<std::vector<int>> clusters;
	/*Create a flag for each point in the cloud, to identified if the point is processed or not, and set it to false*/
	std::vector<bool> processed_flag(cloud->points.size(),false);

	/*Loop through each point of the cloud*/
	for(int idx=0;idx<cloud->points.size();idx++)
	{
		/*Pass the point to Proximity function only if it was not processed
		 * (either added to a cluster or discarded)*/
		if(processed_flag[idx]==false)
		{
			std::vector<int> cluster;
			/*Call Proximity function to identify all the points that are
			 * within in distanceTol distance*/
			Proximity(cloud, cluster,processed_flag,idx,tree,distanceTol,maxSize);
			/*Check if the number of points in the identified cluster are with in limits */
			if((cluster.size()>=minSize)&&cluster.size()<=maxSize)
				clusters.push_back(cluster);
			/*else
				std::cerr<<"discarted cluster"<<cluster.size()<<std::endl;*/
		}

	}
	/*std::cout<<"Distance Tolerance"<<distanceTol<<std::endl;
	std::cout<<"Max Distance "<<tree->max_distance<<std::endl;*/
	return clusters;

}

/* Clustering_euclideanCluster function shall identify the cluster of point that have given
 * no of min, max points and meet the cluster tolerance requirement.
 * Algo: Using points in the given cloud KDTree is formed.
 * 		 Using Euclidena Clustering, clusters are searched in the created KDTree
 * 		 Identified clusters are filtered, clusters that dont have points in min, max points are discarded.
 * */
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering_euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // Create the KdTree object using the points in cloud.
    typename KdTree_euclidean<PointT>::KdTree_euclidean *tree =new KdTree_euclidean<PointT>;
    tree->insert_cloud(cloud);

    //perform euclidean clustering to group detected obstacles
	std::vector<std::vector<int>> cluster_indices = euclideanCluster(cloud, tree,clusterTolerance ,minSize,maxSize);

	for (std::vector<std::vector<int>>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	  {
		typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
	    for (std::vector<int>::const_iterator pit = it->begin (); pit != it->end (); ++pit)
	      cloud_cluster->points.push_back (cloud->points[*pit]); //*
	    cloud_cluster->width = cloud_cluster->points.size ();
	    cloud_cluster->height = 1;
	    cloud_cluster->is_dense = true;

	    clusters.push_back(cloud_cluster);
	  }
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "euclideanClustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

/*BoundingBox function shall identify the min and max coordinates
 *in the provided cluster, a box shall be fitted using these min
 *and max coordinates
 * */
template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    /*Get min and max coordinates in the cluster*/
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

	/*std::cout << "Max x: " << maxPoint.x << std::endl;
	std::cout << "Max y: " << maxPoint.y << std::endl;
	std::cout << "Max z: " << maxPoint.z << std::endl;
	std::cout << "Min x: " << minPoint.x << std::endl;
	std::cout << "Min y: " << minPoint.y << std::endl;
	std::cout << "Min z: " << minPoint.z << std::endl;*/

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
