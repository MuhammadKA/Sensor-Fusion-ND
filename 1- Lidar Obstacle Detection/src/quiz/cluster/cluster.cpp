/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting
#include "cluster.h"
// Arguments:
// window is the region to draw box around
// increase zoom to see more of the area
pcl::visualization::PCLVisualizer::Ptr initScene(Box window, int zoom) {
  pcl::visualization::PCLVisualizer::Ptr viewer(
      new pcl::visualization::PCLVisualizer("2D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  viewer->initCameraParameters();
  viewer->setCameraPosition(0, 0, zoom, 0, 1, 0);
  viewer->addCoordinateSystem(1.0);

  viewer->addCube(window.x_min, window.x_max, window.y_min, window.y_max, 0, 0,
                  1, 1, 1, "window");
  return viewer;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
CreateData(std::vector<std::vector<float>> points) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZ>());

  for (int i = 0; i < points.size(); i++) {
    pcl::PointXYZ point;
    point.x = points[i][0];
    point.y = points[i][1];
    point.z = 0;

    cloud->points.push_back(point);
  }
  cloud->width = cloud->points.size();
  cloud->height = 1;

  return cloud;
}

void render2DTree(Node *node, pcl::visualization::PCLVisualizer::Ptr &viewer,
                  Box window, int &iteration, uint depth = 0) {

  if (node != NULL) {
    Box upperWindow = window;
    Box lowerWindow = window;
    // split on x axis
    if (depth % 2 == 0) {
      viewer->addLine(pcl::PointXYZ(node->point[0], window.y_min, 0),
                      pcl::PointXYZ(node->point[0], window.y_max, 0), 0, 0, 1,
                      "line" + std::to_string(iteration));
      lowerWindow.x_max = node->point[0];
      upperWindow.x_min = node->point[0];
    }
    // split on y axis
    else {
      viewer->addLine(pcl::PointXYZ(window.x_min, node->point[1], 0),
                      pcl::PointXYZ(window.x_max, node->point[1], 0), 1, 0, 0,
                      "line" + std::to_string(iteration));
      lowerWindow.y_max = node->point[1];
      upperWindow.y_min = node->point[1];
    }
    iteration++;

    render2DTree(node->left, viewer, lowerWindow, iteration, depth + 1);
    render2DTree(node->right, viewer, upperWindow, iteration, depth + 1);
  }
}

void clusterHelpter(int indice, const std::vector<std::vector<float>> &points,
                    std::vector<int> &cluster, std::vector<bool> processed,
                    KdTree *tree, float distanceTol) {
  processed[indice] = true;
  cluster.push_back(indice);
  std::vector<int> nearest = tree->search(points[indice], distanceTol);

  for (int id : nearest) {
    if (!processed[id])
      clusterHelpter(id, points, cluster, processed, tree, distanceTol);
  }
}

std::vector<std::vector<int>>
euclideanCluster(const std::vector<std::vector<float>> &points, KdTree *tree,
                 float distanceTol) {

  // TODO: Fill out this function to return list of indices for each cluster

  std::vector<std::vector<int>> clusters;

  std::vector<bool> processed(points.size(),
                              false); // used to keep track of processed points

  int i = 0;
  while (i < points.size()) {
    if (processed[i]) {
      i++;
      continue;
    }
    std::vector<int> cluster;
    clusterHelpter(i, points, cluster, processed, tree, distanceTol);
    clusters.push_back(cluster);
    i++;
  }

  std::cout << "No. of clusters " << clusters.size() << endl;
  return clusters;
}