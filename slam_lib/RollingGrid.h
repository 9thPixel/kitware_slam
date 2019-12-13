#ifndef ROLLING_GRID_H
#define ROLLING_GRID_H

#include "LidarPoint.h"

#define SetMacro(name,type) void Set##name (type _arg) { name = _arg; }
#define GetMacro(name,type) type Get##name () const { return name; }

/*!
 * @brief Rolling voxel grid to store and access pointclouds of specific areas.
 *
 * The map reconstructed from the SLAM algorithm is stored in a voxel grid
 * which splits the space in differents regions. From this voxel grid, it is
 * possible to only load the parts of the map which are pertinents when we run
 * the mapping optimization algorithm. Morevover, when a region of the space is
 * too far from the current sensor position, it is possible to remove the points
 * stored in this region and to move the voxel grid in a closest region of the
 * sensor position. This is used to decrease the memory used by the algorithm.
 */
class RollingGrid
{
public:

  // Usefull types
  using Point = PointXYZTIId;
  using PointCloud = pcl::PointCloud<Point>;

  RollingGrid();

  RollingGrid(double posX, double posY, double posZ);

  //! Roll the grid to enable adding new point cloud
  void Roll(const Eigen::Matrix<double, 6, 1>& T);

  //! Get points near T
  PointCloud::Ptr Get(const Eigen::Matrix<double, 6, 1>& T);

  //! Get all points
  PointCloud::Ptr Get();

  //! Add some points to the grid
  void Add(const PointCloud::Ptr& pointcloud);

  // Remove all points from all voxels
  void Clear();

  void SetPointCoudMaxRange(double maxdist);

  void SetGridSize(int size);
  GetMacro(GridSize, int)

  SetMacro(VoxelResolution, double)
  GetMacro(VoxelResolution, double)

  SetMacro(LeafSize, double)
  GetMacro(LeafSize, double)

private:

  //! [voxels] Size of the voxel grid: n*n*n voxels
  int GridSize = 50;

  //! [m/voxel] Resolution of a voxel
  double VoxelResolution = 10.;

  //! [voxels] Size of the current added pointcloud in voxels
  int PointCloudSize = 25;

  //! [m] Size of the leaf used to downsample the pointcloud with a VoxelGrid filter
  double LeafSize = 0.2;

  //! VoxelGrid of pointcloud
  std::vector<std::vector<std::vector<PointCloud::Ptr>>> Grid;

  //! [voxel, voxel, voxel] Position of the VoxelGrid
  int VoxelGridPosition[3] = { 0, 0, 0 };
};

#endif  // ROLLING_GRID_H