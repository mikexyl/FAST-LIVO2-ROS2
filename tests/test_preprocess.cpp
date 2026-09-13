#include "preprocess.h"
#include <pcl/filters/voxel_grid.h>
#include <cmath>
#include <iostream>

int main()
{
  Preprocess processor;
  processor.lidar_type = VELO16;
  processor.N_SCANS = 16;
  processor.blind = 1.0;
  processor.velodyne_time_scale = 1000.0;

  pcl::PointCloud<velodyne_ros::Point> points;
  for (int i = 0; i < 3; ++i)
  {
    velodyne_ros::Point point{};
    point.x = i == 0 ? 0.5f : static_cast<float>(i + 2);
    point.intensity = 10.0f;
    point.time = i * 0.05f;
    points.push_back(point);
  }
  auto message = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(points, *message);
  auto output = std::make_shared<PointCloudXYZI>();
  processor.process(message, output);
  if (output->size() != 2 || output->front().x != 3.0f ||
      std::abs(output->front().curvature - 50.0f) > 1e-4f ||
      std::abs(output->back().curvature - 100.0f) > 1e-4f)
  {
    std::cerr << "Blind range or seconds-to-milliseconds conversion failed\n";
    return 1;
  }

  processor.blind = 3.5;
  processor.process(message, output);
  if (output->size() != 1 || output->front().x != 4.0f)
  {
    std::cerr << "Updated blind range was not applied\n";
    return 1;
  }

  // PCL's binary allocates the output; our Eigen allocator must free it safely.
  pcl::VoxelGrid<PointType> filter;
  filter.setLeafSize(0.2f, 0.2f, 0.2f);
  filter.setInputCloud(output);
  PointCloudXYZI filtered;
  filter.filter(filtered);
  if (filtered.size() != 1 || filtered.front().x != 4.0f)
  {
    std::cerr << "PCL voxel filter did not preserve the test point\n";
    return 1;
  }
  filtered.points.clear();
  filtered.points.shrink_to_fit();
  return 0;
}
