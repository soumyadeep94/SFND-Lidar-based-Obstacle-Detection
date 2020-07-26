// PCL lib Functions for processing point clouds

#include "processPointClouds.h"


using namespace Eigen;
//constructor:
template <typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}

//de-constructor:
template <typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}

template <typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>());
    pcl::VoxelGrid<PointT> vox;
    vox.setInputCloud(cloud);
    vox.setLeafSize(filterRes, filterRes, filterRes);
    vox.filter(*cloud_filtered);

    typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>());
    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloud_filtered);
    region.filter(*cloudRegion);

    std::vector<int> indices;

    pcl::CropBox<PointT> roof(true);
    roof.setMin(Vector4f(-1.5, -1.7, -1, 1));
    roof.setMax(Vector4f(2.6, 1.7, -0.4, 1));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    for (int point : indices){
        inliers->indices.push_back(point);
    }

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
void ProcessPointClouds<PointT>::proximity(int index, const std::vector<std::vector<float>>& points, std::vector<int>& cluster, KdTree* tree, std::vector<bool>& processed, float clusterTol) {
    processed[index] = true;
    cluster.push_back(index);
    std::vector<int> nearest = tree->search(points[index], clusterTol);

    for (int idx : nearest) {
        if(!processed[idx]){
            proximity(idx, points, cluster, tree, processed, clusterTol);
        }
    }
}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclidean_cluster(const std::vector<std::vector<float>>& points, KdTree* tree, float clusterTol) {
    std::vector<std::vector<int>>clusters;
    std::vector<bool>processed(points.size(), false);
    for (int i=0; i<points.size(); i++) {
        if(processed[i]) {
            continue;
        }
        std::vector<int>cluster;
        proximity(i, points, cluster, tree, processed, clusterTol);
        clusters.push_back(cluster);
    }

    return clusters;

}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering3D(typename pcl::PointCloud<PointT>::Ptr cloud, double clusterTol, int minSize, int maxSize) {
    
    auto startTime = std::chrono::steady_clock::now();
    KdTree* tree = new KdTree;
    std::vector<std::vector<float>> vecPoints;
    // vecPoints.reserve(cloud->points.size());
    for (int i = 0; i < cloud->points.size(); ++i) { 
        std::vector<float>vPoint{cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};     
        tree->insert(vPoint, i); 
        vecPoints.push_back(vPoint);
    }


    std::vector<std::vector<int>> clusters = euclidean_cluster(vecPoints, tree, clusterTol);
    std::cout<<"no. of euclidean clusters: "<<clusters.size()<<std::endl;
    std::vector<typename pcl::PointCloud<PointT>::Ptr> resultClusters;

    for (int i=0; i<clusters.size(); i++) {
        typename pcl::PointCloud<PointT>::Ptr pcl_cluster (new pcl::PointCloud<PointT>());
        for (int j=0; j<clusters[i].size(); j++) {
            if (clusters[i].size() < minSize || clusters[i].size() > maxSize){
                continue;
            }
            pcl_cluster->points.push_back(cloud->points[clusters[i][j]]);
        }

        pcl_cluster->width = pcl_cluster->points.size();
        pcl_cluster->height = 1;
        pcl_cluster->is_dense = true;
        if (pcl_cluster->points.size() > 0)
            resultClusters.push_back(pcl_cluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " microseconds and found " << resultClusters.size() << " clusters" << std::endl;

    return resultClusters;



}


template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, double clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    std::vector<typename pcl::PointIndices> cluster_indices;

    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    tree->setInputCloud(cloud);
    pcl::EuclideanClusterExtraction<PointT> ec;
    
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    for (auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
        typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>());
        for (auto pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
            cluster->points.push_back(cloud->points[*pit]);
        }
        cluster->width = cluster->points.size();
        cluster->height = 1;
        cluster->is_dense = true;
        clusters.push_back(cluster);
    }

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " microseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template <typename PointT>
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


template <typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII(file, *cloud);
    std::cerr << "Saved " << cloud->points.size() << " data points to " + file << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size() << " data points from " + file << std::endl;

    return cloud;
}

template <typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>ProcessPointClouds<PointT>::Ransac_plane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	auto startTime = std::chrono::steady_clock::now();
	std::unordered_set<int> inliersResult;
    typename pcl::PointCloud<PointT>::Ptr cloudInliers(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());
	srand(time(NULL));

	while (--maxIterations)
	{
		std::unordered_set<int> inliers;

		int idx1 = rand() % cloud->points.size();
		int idx2 = rand() % cloud->points.size();
		int idx3 = rand() % cloud->points.size();
		
		inliers.insert(idx1);
		inliers.insert(idx2);
		inliers.insert(idx3);

		auto P = cloud->points[idx1];
		auto Q = cloud->points[idx2];
		auto R = cloud->points[idx3];

        
		Vector3d PQ(Q.x - P.x, Q.y-P.y, Q.z - P.z);
		Vector3d PR(R.x - P.x, R.y - P.y, R.z - P.z);
		Vector3d normal = PQ.cross(PR);
		normal.normalize();

		double A = normal(0);
		double B = normal(1);
		double C = normal(2);

		double D = -normal.dot(Vector3d(P.x, P.y, P.z));


		//std::cout<<"size of cloud: "<<cloud->points.size()<<std::endl;
		for (int idx = 0; idx < cloud->points.size(); ++idx)
		{
			if (inliers.count(idx) > 0){
				continue;
			}

			double x1 = cloud->points[idx].x;
			double y1 = cloud->points[idx].y;
			double z1 = cloud->points[idx].z;


			double distance = fabs(A*x1 + B*y1 + C*z1 + D)/sqrt(A*A + B*B + C*C);
			
			if (distance < distanceTol)
			{	
				// std::cout<<"distance to the plane is: "<<distance<<std::endl;
				inliers.insert(idx);
			}
		}
		if (inliers.size() > inliersResult.size())
		{
			inliersResult = std::move(inliers);
			std::cout << std::endl;
		}
	}
    std::cout << "No. of inliers: " << inliersResult.size() << std::endl;

    for (int idx = 0; idx < cloud->points.size(); ++idx) {
        PointT point = cloud->points[idx];
        if (inliersResult.count(idx)) {
            cloudInliers->points.push_back(point);
        }
        else {
            cloudOutliers->points.push_back(point);
        }
    }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> resultantCloud(cloudOutliers, cloudInliers);
	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
	std::cout << "The line fitting using RANSAC took " << elapsedTime.count() << " microseconds" << std::endl;

    auto segOutliers = resultantCloud.first;
    auto segInliers = resultantCloud.second;

    std::cout<<"size of the outliers: "<<segOutliers->points.size()<<std::endl;
    std::cout<<"size of the inliers: "<<segInliers->points.size()<<std::endl;

	return resultantCloud;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) {
    typename pcl::PointCloud<PointT>::Ptr obsCloud (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr roadCloud (new pcl::PointCloud<PointT>());
     for (auto idx : inliers->indices) {
         roadCloud->points.push_back(cloud->points[idx]);
     }
     pcl::ExtractIndices<PointT> extract;
     extract.setInputCloud(cloud);
     extract.setIndices(inliers);
     extract.setNegative(true);
     extract.filter(*obsCloud);

     std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult (obsCloud, roadCloud);

     return segResult;
 }



template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    auto startTime = std::chrono::steady_clock::now();
    // pcl::PointIndices::Ptr pclInliers (new pcl::PointIndices ());

    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients());

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    // pcl::PointIndices::Ptr inliers = Ransac_plane(cloud, maxIterations, distanceThreshold);
    

    if (inliers->indices.size() == 0) {
        std::cout<<"Could not estimate a planar model for the given dataset"<<std::endl;

    }
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " microseconds" << std::endl;
    auto segOutliers = segResult.first;
    auto segInliers = segResult.second;

    std::cout<<"size of the outliers: "<<segOutliers->points.size()<<std::endl;
    std::cout<<"size of the inliers: "<<segInliers->points.size()<<std::endl;
    return segResult;
}

