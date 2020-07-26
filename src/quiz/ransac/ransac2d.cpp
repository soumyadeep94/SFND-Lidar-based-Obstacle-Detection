/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include <math.h>
#include<eigen3/Eigen/Dense>

using namespace Eigen;

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	// Add inliers
	float scatter = 0.6;
	for (int i = -5; i < 5; i++)
	{
		double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
		double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
		pcl::PointXYZ point;
		point.x = i + scatter * rx;
		point.y = i + scatter * ry;
		point.z = 0;

		cloud->points.push_back(point);
	}
	// Add outliers
	int numOutliers = 10;
	while (numOutliers--)
	{
		double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
		double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
		pcl::PointXYZ point;
		point.x = 5 * rx;
		point.y = 5 * ry;
		point.z = 0;

		cloud->points.push_back(point);
	}
	cloud->width = cloud->points.size();
	cloud->height = 1;

	return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}

pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("2D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->initCameraParameters();
	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
	viewer->addCoordinateSystem(1.0);
	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	auto startTime = std::chrono::steady_clock::now();
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	while (--maxIterations)
	{
		std::unordered_set<int> inliers;
		int idx1 = rand() % cloud->points.size();
		int idx2 = rand() % cloud->points.size();
		std::cout << "pivot id1: " << idx1 << ","
				  << "pivot id2: " << idx2 << std::endl;
		inliers.insert(idx1);
		inliers.insert(idx2);

		auto inPoints1 = cloud->points[idx1];
		auto inPoints2 = cloud->points[idx2];

		float A = inPoints1.y - inPoints2.y;
		float B = inPoints2.x - inPoints1.x;
		float C = inPoints1.x * inPoints2.y - inPoints2.x * inPoints1.y;
		
		//std::cout<<"size of cloud: "<<cloud->points.size()<<std::endl;
		for (int idx = 0; idx < cloud->points.size(); ++idx)
		{
			if (inliers.count(idx) > 0){
				continue;
			}

			int x1 = cloud->points[idx].x;
			int y1 = cloud->points[idx].y;

			float distance = fabs((A * x1 + B * y1 + C) / (sqrt(A * A + B * B)));
			if (distance <= distanceTol)
			{
				inliers.insert(idx);
			}
		};
		if (inliers.size() > inliersResult.size())
		{
			inliersResult = std::move(inliers);
			std::cout << std::endl;
		}
	}
	std::cout << "No. of inliers: " << inliersResult.size() << std::endl;
	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
	std::cout << "The line fitting using RANSAC took " << elapsedTime.count() << " microseconds" << std::endl;


	// TODO: Fill in this function

	// For max iterations

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers

	return inliersResult;
}

std::unordered_set<int> Ransac_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	auto startTime = std::chrono::steady_clock::now();
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	while (--maxIterations)
	{
		std::unordered_set<int> inliers;

		int idx1 = rand() % cloud->points.size();
		int idx2 = rand() % cloud->points.size();
		int idx3 = rand() % cloud->points.size();
		
		std::cout << "pivot id1: " << idx1 << ","
				  << "pivot id2: " << idx2 <<", " 
				  << "pivot id3: "<<idx3<<std::endl;
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
		std::cout<<"Vector PQ: "<<PQ(0)<<" "<<PQ(1)<<" "<<PQ(2)<<std::endl;
		std::cout<<"Vector PR: "<<PR(0)<<" "<<PR(1)<<" "<<PR(2)<<std::endl;
		std::cout<<"normals are:"<<A<<","<<B<<","<<C<<std::endl;

		double D = -normal.dot(Vector3d(P.x, P.y, P.z));


		//std::cout<<"size of cloud: "<<cloud->points.size()<<std::endl;
		for (int idx = 0; idx < cloud->points.size(); ++idx)
		{
			if (inliers.count(idx) > 0){
				continue;
			}

			int x1 = cloud->points[idx].x;
			int y1 = cloud->points[idx].y;
			int z1 = cloud->points[idx].z;


			double distance = fabs((A*x1 + B*y1 + C*z1 + D))/sqrt(A*A + B*B + C*C);
			
			if (distance <= distanceTol)
			{	
				std::cout<<"distance to the plane is: "<<distance<<std::endl;
				inliers.insert(idx);
			}
		};
		if (inliers.size() > inliersResult.size())
		{
			inliersResult = std::move(inliers);
			std::cout << std::endl;
		}
	}
	std::cout << "No. of inliers: " << inliersResult.size() << std::endl;
	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
	std::cout << "The line fitting using RANSAC took " << elapsedTime.count() << " microseconds" << std::endl;


	// TODO: Fill in this function

	// For max iterations

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers

	return inliersResult;
}

int main()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	pcl::PointIndices::Ptr pclInliers (new pcl::PointIndices());

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac_plane(cloud, 50, 0.2);
	std::cout << "index of the point: ";
	for (auto idx : inliers)
	{
		pclInliers->indices.push_back(idx);
		std::cout << idx << " ";
	}
	std::cout << std::endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for (int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if (inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}

	// Render 2D point cloud with inliers and outliers
	if (inliers.size())
	{
		renderPointCloud(viewer, cloudInliers, "inliers", Color(0, 1, 0));
		renderPointCloud(viewer, cloudOutliers, "outliers", Color(1, 0, 0));
	}
	else
	{
		renderPointCloud(viewer, cloud, "data");
	}

	while (!viewer->wasStopped())
	{
		viewer->spinOnce();
	}
}
