// PCL lib Functions for processing point clouds

#include "processPointClouds.h"
#include "cluster.h"
#include <unordered_set>

// constructor:
template <typename PointT> ProcessPointClouds<PointT>::ProcessPointClouds() {}

// de-constructor:
template <typename PointT> ProcessPointClouds<PointT>::~ProcessPointClouds() {}

template <typename PointT>
void ProcessPointClouds<PointT>::numPoints(
    typename pcl::PointCloud<PointT>::Ptr cloud) {
  std::cout << cloud->points.size() << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(
    typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes,
    Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint) {

  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();

  // TODO:: Fill in the function to do voxel grid point reduction and region
  // based filtering
  typename pcl::PointCloud<pcl::PointXYZI>::Ptr cloudFiltered(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::VoxelGrid<PointT> vg;
  vg.setInputCloud(cloud);
  vg.setLeafSize(filterRes, filterRes, filterRes);
  vg.filter(*cloudFiltered);

  typename pcl::PointCloud<pcl::PointXYZI>::Ptr cloudRegion(
      new pcl::PointCloud<pcl::PointXYZI>);

  pcl::CropBox<PointT> region(true);
  /*you set the region to true because you are dealing with points inside the
   * crop box*/
  region.setMin(minPoint);
  region.setMax(maxPoint);
  region.setInputCloud(cloudFiltered);
  region.filter(*cloudRegion);
  /*Now we are only left with points inside the box region*/

  // Now, we want to do the opposite of what we did. We want to remove
  // reflections from the roof of ego vehicle so we want to define a box (ego
  // car) and remove what is in it, and keep everything outside.

  std::vector<int> indices;

  pcl::CropBox<PointT> roof(true);
  /*you set the region to true because you are dealing with points inside the
   * crop box*/
  roof.setMin(Eigen::Vector4f(-1.5, -1.5, -2, 1));
  roof.setMax(Eigen::Vector4f(3, 3, 2, 1));
  roof.setInputCloud(cloudRegion);
  roof.filter(indices);

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  for (int point : indices)
    inliers->indices.push_back(point);

  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloudRegion);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*cloudRegion);

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "filtering took " << elapsedTime.count() << " milliseconds"
            << std::endl;

  return cloudRegion;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
          typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::SeparateClouds(
    pcl::PointIndices::Ptr inliers,
    typename pcl::PointCloud<PointT>::Ptr cloud) {
  // TODO: Create two new point clouds, one cloud with obstacles and other with
  // segmented plane

  typename pcl::PointCloud<PointT>::Ptr obstCloud(
      new pcl::PointCloud<PointT>());
  typename pcl::PointCloud<PointT>::Ptr planeCloud(
      new pcl::PointCloud<PointT>());

  for (int index : inliers->indices)
    planeCloud->points.push_back(cloud->points[index]);

  pcl::ExtractIndices<PointT> extract;

  extract.setInputCloud(cloud); // give the reference cloud
  extract.setIndices(inliers);  // here are the inliers
  extract.setNegative(true);    // set the inliers to negative
  extract.filter(*obstCloud); // then dereference the obstacle cloud since it is
                              // a pointer so the filter function will keep all
                              // the points that are not inliers

  std::pair<typename pcl::PointCloud<PointT>::Ptr,
            typename pcl::PointCloud<PointT>::Ptr>
      segResult(obstCloud, planeCloud);

  return segResult;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
          typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::SegmentPlane(
    typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations,
    float distanceTol) {

  auto startTime = std::chrono::steady_clock::now();
  std::unordered_set<int> inliersResult;

  // ransac
  while (maxIterations--) {
    // Randomly sample subset and fit line
    std::unordered_set<int> curr_inliers;
    while (curr_inliers.size() != 3) {
      curr_inliers.insert(rand() % cloud->points.size());
    }

    auto itr = curr_inliers.begin();
    float x1 = cloud->points[*itr].x;
    float y1 = cloud->points[*itr].y;
    float z1 = cloud->points[*itr].z;
    itr++;
    float x2 = cloud->points[*itr].x;
    float y2 = cloud->points[*itr].y;
    float z2 = cloud->points[*itr].z;
    itr++;
    float x3 = cloud->points[*itr].x;
    float y3 = cloud->points[*itr].y;
    float z3 = cloud->points[*itr].z;

    // plane params
    float a = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
    float b = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
    float c = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
    float d = -(a * x1 + b * y1 + c * z1);

    // Measure distance between every point and fitted line
    for (int i = 0; i < cloud->points.size(); ++i) {
      if (curr_inliers.count(i) > 0)
        continue;

      float nx = cloud->points[i].x;
      float ny = cloud->points[i].y;
      float nz = cloud->points[i].z;
      float dist =
          fabs(a * nx + b * ny + c * nz + d) / sqrt(a * a + b * b + c * c);

      if (dist < distanceTol)
        curr_inliers.insert(i);
    }

    if (curr_inliers.size() > inliersResult.size())
      inliersResult = curr_inliers;
  }

  // inliersResult
  typename pcl::PointCloud<PointT>::Ptr cloudInliers(
      new pcl::PointCloud<PointT>());
  typename pcl::PointCloud<PointT>::Ptr cloudOutliers(
      new pcl::PointCloud<PointT>());

  for (int index = 0; index < cloud->points.size(); index++) {
    PointT point = cloud->points[index];
    if (inliersResult.count(index))
      cloudInliers->points.push_back(point);
    else
      cloudOutliers->points.push_back(point);
  }

  std::pair<typename pcl::PointCloud<PointT>::Ptr,
            typename pcl::PointCloud<PointT>::Ptr>
      segResult(cloudOutliers, cloudInliers);

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "plane segmentation took " << elapsedTime.count()
            << " milliseconds" << std::endl;

  return segResult;
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::Clustering(
    typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance,
    int minSize, int maxSize) {

  // Time clustering process
  auto startTime = std::chrono::steady_clock::now();

  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

  // kd tree
  KdTree *tree = new KdTree;
  std::vector<std::vector<float>> points;
  for (int i = 0; i < cloud->points.size(); i++) {
    PointT point = cloud->points[i];

    std::vector<float> point_vector;
    point_vector.push_back(point.x);
    point_vector.push_back(point.y);
    point_vector.push_back(point.z);

    tree->insert(point_vector, i);
    points.push_back(point_vector);
  }

  // cluster
  std::vector<std::vector<int>> clusters_idx =
      euclideanCluster(points, tree, clusterTolerance);

  for (std::vector<int> cluster_idx : clusters_idx) {
    typename pcl::PointCloud<PointT>::Ptr clusterCloud(
        new pcl::PointCloud<PointT>());
    for (int indice : cluster_idx) {
      clusterCloud->points.push_back(cloud->points[indice]);
    }
    clusterCloud->width = clusterCloud->points.size();
    clusterCloud->height = 1;
    clusterCloud->is_dense = true;
    if ((clusterCloud->width >= minSize) and (clusterCloud->width <= maxSize))
      clusters.push_back(clusterCloud);
  }

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "clustering took " << elapsedTime.count()
            << " milliseconds and found " << clusters.size() << " clusters"
            << std::endl;

  return clusters;
}

template <typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(
    typename pcl::PointCloud<PointT>::Ptr cluster) {

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
void ProcessPointClouds<PointT>::savePcd(
    typename pcl::PointCloud<PointT>::Ptr cloud, std::string file) {
  pcl::io::savePCDFileASCII(file, *cloud);
  std::cerr << "Saved " << cloud->points.size() << " data points to " + file
            << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr
ProcessPointClouds<PointT>::loadPcd(std::string file) {

  typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

  if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1) //* load the file
  {
    PCL_ERROR("Couldn't read file \n");
  }
  std::cerr << "Loaded " << cloud->points.size() << " data points from " + file
            << std::endl;

  return cloud;
}

template <typename PointT>
std::vector<boost::filesystem::path>
ProcessPointClouds<PointT>::streamPcd(std::string dataPath) {

  std::vector<boost::filesystem::path> paths(
      boost::filesystem::directory_iterator{dataPath},
      boost::filesystem::directory_iterator{});

  // sort files in accending order so playback is chronological
  sort(paths.begin(), paths.end());

  return paths;
}