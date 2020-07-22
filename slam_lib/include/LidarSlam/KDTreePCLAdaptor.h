//==============================================================================
// Copyright 2019-2020 Kitware, Inc., Kitware SAS
// Author: Guilbert Pierre (Kitware SAS)
// Creation date: 2018-04-19
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//==============================================================================

#ifndef KDTREE_PCL_ADAPTOR_H
#define KDTREE_PCL_ADAPTOR_H

#include <nanoflann.hpp>
#include <pcl/point_cloud.h>

template<typename PointT>
class KDTreePCLAdaptor
{
  using Point = PointT;
  using PointCloud = pcl::PointCloud<Point>;
  using PointCloudPtr = typename PointCloud::Ptr;

  using metric_t = typename nanoflann::metric_L2::template traits<double, KDTreePCLAdaptor<Point>>::distance_t;
  using index_t = nanoflann::KDTreeSingleIndexAdaptor<metric_t, KDTreePCLAdaptor<Point>, 3, int>;

public:

  KDTreePCLAdaptor() = default;

  KDTreePCLAdaptor(PointCloudPtr cloud)
  {
    this->Reset(cloud);
  }

  void Reset(PointCloudPtr cloud)
  {
    // copy the input cloud
    this->Cloud = cloud;

    // Build KD-tree
    int leafMaxSize = 25;
    Index = std::make_unique<index_t>(3, *this, nanoflann::KDTreeSingleIndexAdaptorParams(leafMaxSize));
    Index->buildIndex();
  }

  /** Query for the \a num_closest closest points to a given point (entered as query_point[0:dim-1]).
    *  Note that this is a short-cut method for index->findNeighbors().
    *  The user can also call index->... methods as desired.
    * \note nChecks_IGNORED is ignored but kept for compatibility with the original FLANN interface.
    */
  inline void query(const Point& query_point, int knearest, int* out_indices, double* out_distances_sq/*, const int nChecks_IGNORED = 10*/) const
  {
    double pt[3] = {query_point.x, query_point.y, query_point.z};
    nanoflann::KNNResultSet<double, int> resultSet(knearest);
    resultSet.init(out_indices, out_distances_sq);
    this->Index->findNeighbors(resultSet, pt, nanoflann::SearchParams());
  }
  inline void query(const double query_point[3], int knearest, int* out_indices, double* out_distances_sq/*, const int nChecks_IGNORED = 10*/) const
  {
    nanoflann::KNNResultSet<double, int> resultSet(knearest);
    resultSet.init(out_indices, out_distances_sq);
    this->Index->findNeighbors(resultSet, query_point, nanoflann::SearchParams());
  }

  inline const KDTreePCLAdaptor& derived() const
  {
    return *this;
  }

  inline KDTreePCLAdaptor& derived()
  {
    return *this;
  }

  // Must return the number of data points
  inline int kdtree_get_point_count() const
  {
    return this->Cloud->size();
  }

  // Returns the dim'th component of the idx'th point in the class:
  inline double kdtree_get_pt(const int idx, const int dim) const
  {
    if (dim == 0)
      return this->Cloud->points[idx].x;
    else if (dim == 1)
      return this->Cloud->points[idx].y;
    else
      return this->Cloud->points[idx].z;
  }

  inline PointCloudPtr getInputCloud() const
  {
    return this->Cloud;
  }

  // Optional bounding-box computation: return false to default to a standard bbox computation loop.
  //   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
  //   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
  template <class BBOX>
  inline bool kdtree_get_bbox(BBOX& /*bb*/) const
  {
    return false;
  }

protected:

  //! The kd-tree index for the user to call its methods as usual with any other FLANN index.
  std::unique_ptr<index_t> Index;

  //! The input data
  PointCloudPtr Cloud;
};

#endif // KDTREE_PCL_ADAPTOR_H