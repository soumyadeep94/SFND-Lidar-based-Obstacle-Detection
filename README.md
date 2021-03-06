# Lidar-based-Obstacle-Detection

<img src="media/ObstacleDetectionFPS.gif" width="700" height="400" />

This project has been built in the course of **Udacity Sensor Fusion Nanodegree Program**.
The main objective of this project is to preprocess, cluster and draw bounding boxes around obstacles in a sequence of lidar point cloud data using custom algorithms implemented using PCL(Point Cloud Library) in C++.

**Workflow

**Filtering and Cropping**: This is the initial stage of the lidar preprocessing pipeline. Most of the task here involves downsampling point cloud data to reduce the number of points for fast processing. We use PCL implementation of voxel grid filtering to downsample the cloud data. Another important point of this stage is to remove cloud points which are beyond the sides of the road.

**Segmentation**: At this stage, we use our custom C++ implementation of the Random Sample Consensus (RANSAC) algorithm to separate the ground plane from the obstacle plane. More details: https://en.wikipedia.org/wiki/Random_sample_consensus

**Clustering** At this stage also, we use our custom C++ implementation of the Kd-tree data structure to efficiently extract clusters from the obstacle plane. We then draw bounding boxes around each cluster which represents an obstacle within our ego vehichle environment.


## Installation

### Ubuntu 

```bash
$> sudo apt install libpcl-dev
$> cd ~
$> git clone https://github.com/soumyadeep94/SFND_Lidar_Obstacle_Detection.git
$> cd SFND_Lidar_Obstacle_Detection
$> mkdir build && cd build
$> cmake ..
$> make
$> ./environment
```
