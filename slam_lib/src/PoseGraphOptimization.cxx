#include "LidarSlam/PoseGraphOptimization.h"
#include "LidarSlam/GlobalTrajectoriesRegistration.h"

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/slam3d/types_slam3d.h>

namespace Eigen
{
  using Matrix6d = Eigen::Matrix<double, 6, 6>;
}

namespace
{
  //----------------------------------------------------------------------------
  Eigen::Isometry3d GetRelativeTransform(const Transform& pose1, const Transform& pose2)
  {
    return pose1.GetIsometry().inverse() * pose2.GetIsometry();
  }

  //----------------------------------------------------------------------------
  /*!
   * @brief Find closest SLAM point index matching with a GPS point.
   * @param[in] gpsPose     The GPS point to match.
   * @param[in] slamPoses   The SLAM points to search in.
   * @param[in] maxTimeDiff The maximum time difference allowed between a
   *                        GPS/SLAM pair of points to be considered matching.
   * @return The index of the closest SLAM point.
   *
   * NOTE : The assertion of sorted vectors is done (time is increasing through poses).
   */
  int FindClosestSlamPose(const Transform& gpsPose, const std::vector<Transform>& slamPoses, double maxTimeDiff = 0.1)
  {
    double gpsTime = gpsPose.time;
    double timeDiff, prevTimeDiff = std::numeric_limits<double>::max();
    int bestId = -1;
    int nSlamPoses = slamPoses.size();
    for (int j = 0; j < nSlamPoses; ++j)
    {
      timeDiff = std::abs(gpsTime - slamPoses[j].time);

      if (timeDiff <= maxTimeDiff)
      {
        if (timeDiff < prevTimeDiff)
          bestId = j;
        // CHECK As vector is sorted, if time difference is increasing, we already passed through best point.
        // else
        //   return bestId;        
      }

      prevTimeDiff = timeDiff;
    }
    return bestId;
  }
}


//------------------------------------------------------------------------------
PoseGraphOptimization::PoseGraphOptimization()
{
  // create optimizer
  // TODO change optimizer
  auto linearSolver = std::make_unique<g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>();
  auto* solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver)));
  this->GraphOptimizer.setAlgorithm(solver);
  this->GraphOptimizer.setVerbose(this->Verbose);

  // add sensor/GPS offset parameter
  auto* sensorToGpsCalibration = new g2o::ParameterSE3Offset;
  sensorToGpsCalibration->setId(0);
  this->GraphOptimizer.addParameter(sensorToGpsCalibration);
}

//------------------------------------------------------------------------------
void PoseGraphOptimization::SetGpsToSensorCalibration(double x, double y, double z, double rx, double ry, double rz)
{
  Eigen::Translation3d trans(x, y, z);
  Eigen::Quaterniond rot(Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ()) *
                         Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX()));
  Eigen::Isometry3d transform(trans * rot);
  this->SetGpsToSensorCalibration(transform);
}

//------------------------------------------------------------------------------
void PoseGraphOptimization::SetGpsToSensorCalibration(const Eigen::Isometry3d& gpsToSensor)
{
  auto* sensorToGpsCalibration = dynamic_cast<g2o::ParameterSE3Offset*>(this->GraphOptimizer.parameter(0));
  if (sensorToGpsCalibration)
    sensorToGpsCalibration->setOffset(gpsToSensor.inverse());
  else
    std::cerr << "[ERROR] The first g2o parameter is not an SO3 Offset" << std::endl;
}

//------------------------------------------------------------------------------
bool PoseGraphOptimization::Process(const std::vector<Transform>& slamPoses,
                                    const std::vector<Transform>& gpsPoses,
                                    const std::vector<std::array<double, 36>>& slamCov,
                                    const std::vector<std::array<double, 9>>& gpsCov,
                                    std::vector<Transform>& optimizedSlamPoses)
{
  unsigned int nbSlamPoses = slamPoses.size();
  unsigned int nbGpsPoses = gpsPoses.size();

  // Check input vector sizes (at least 2 elements)
  if ((nbSlamPoses < 2) || (nbGpsPoses < 2))
  {
    std::cerr << "[ERROR] SLAM and GPS trajectories must have at least 2 points "
              << "(Got " << nbSlamPoses << " SLAM poses and " << nbGpsPoses << " GPS positions)."
              << std::endl;
    return false;
  }

  // Check timestamps
  double gpsInitTime = gpsPoses[0].time;
  double gpsEndTime = gpsPoses[nbGpsPoses - 1].time;
  double slamInitTime = slamPoses[0].time;
  double slamEndTime = slamPoses[nbSlamPoses - 1].time;
  auto checkInterval = [](double t, double tmin, double tmax) { return tmin < t && t < tmax; };
  if (!checkInterval(gpsInitTime, slamInitTime, slamEndTime) &&
      !checkInterval(slamInitTime, gpsInitTime, gpsEndTime))
  {
    std::cerr << "[ERROR] Slam time and GPS time do not match. "
              << "GPS = ("<< gpsInitTime << ", " << gpsEndTime << "); "
              << "SLAM = ("<< slamInitTime << ", " << slamEndTime << "). "
              << "Please indicate the time offset." << std::endl;
    return false;
  }

  // Compute global transformation from SLAM to GPS data to get a better initialization.
  GlobalTrajectoriesRegistration registration;
  registration.SetVerbose(this->Verbose);
  Eigen::Isometry3d worldToSlam;
  registration.ComputeTransformOffset(slamPoses, gpsPoses, worldToSlam);

  // Apply transformation to SLAM poses
  std::vector<Transform> transSlamPoses;
  transSlamPoses.reserve(nbSlamPoses);
  for (const Transform& slamPose: slamPoses)
  {
    Eigen::Isometry3d finalSlamPose(worldToSlam * slamPose.GetIsometry());
    transSlamPoses.emplace_back(finalSlamPose, slamPose.time + this->TimeOffset, slamPose.frameid);
  }

  // Clear and build the pose graph to optimize
  this->BuildPoseGraph(transSlamPoses, gpsPoses, slamCov, gpsCov);

  // Save Graph before optimization
  if (this->SaveG2OFile)
  {
    if (!G2OFileName.empty())
      this->GraphOptimizer.save(G2OFileName.c_str());
    else
      std::cerr << "[WARNING] Could not save the g2o graph. Please specify a filename." << std::endl;
  }

  // Print debug info
  if (this->Verbose)
  {
    std::cout << std::endl << "The Graph is composed of:" << std::endl
              << "   " << nbSlamPoses     << " SLAM vertices" << std::endl
              << "   " << nbSlamPoses - 1 << " SLAM edges" << std::endl
              << "   " << this->GraphOptimizer.vertices().size() - nbSlamPoses    << " GPS vertices" << std::endl
              << "   " << this->GraphOptimizer.edges().size() - (nbSlamPoses - 1) << " GPS/SLAM edges" << std::endl;
  }

  // Optimize the graph
  if (!this->GraphOptimizer.initializeOptimization())
  {
    std::cerr << "[ERROR] Pose graph initialization failed !" << std::endl;
    return false;
  }
  int iterations = this->GraphOptimizer.optimize(this->NbIteration);

  // Print debug info if needed
  if (this->Verbose)
    std::cout << "Pose graph optimization succeeded in " << iterations << " iterations." << std::endl << std::endl;

  // Set the output optimized data
  optimizedSlamPoses.clear();
  optimizedSlamPoses.reserve(nbSlamPoses);
  for (unsigned int i = 0; i < nbSlamPoses; ++i)
  {
    // Get optimized SLAM vertex pose
    auto* v = this->GraphOptimizer.vertex(i);
    double data[7];
    v->getEstimateData(data);
    // Fill new optimized trajectory
    Eigen::Translation3d trans(data[0], data[1], data[2]);
    Eigen::Quaterniond rot(data[6], data[3], data[4], data[5]);
    optimizedSlamPoses.emplace_back(trans, rot, transSlamPoses[i].time, transSlamPoses[i].frameid);
  }

  return true;
}

//------------------------------------------------------------------------------
void PoseGraphOptimization::BuildPoseGraph(const std::vector<Transform>& slamPoses,
                                           const std::vector<Transform>& gpsPoses,
                                           const std::vector<std::array<double, 36>>& slamCov,
                                           const std::vector<std::array<double, 9>>& gpsCov)
{
  // Reset the graph
  this->GraphOptimizer.clear();
  int idCount = -1;

  // Handle SLAM data
  for (unsigned int i = 0; i < slamPoses.size(); ++i)
  {
    // Add SLAM pose (position + orientation) as a vertex
    auto* slamVertex = new g2o::VertexSE3;
    slamVertex->setId(++idCount);
    slamVertex->setEstimate(slamPoses[i].GetIsometry());
    slamVertex->setFixed(false);
    this->GraphOptimizer.addVertex(slamVertex);

    // Add edge between 2 consecutive SLAM poses
    if (i > 0)
    {
      // Get edge between two last SLAM poses
      Eigen::Isometry3d relativeTransform = GetRelativeTransform(slamPoses[i-1], slamPoses[i]);
      Eigen::Map<Eigen::Matrix6d> covMatrix((double*) slamCov[i].data());

      // Add edge to pose graph
      auto* slamEdge = new g2o::EdgeSE3;
      slamEdge->setVertex(0, this->GraphOptimizer.vertex(i - 1));
      slamEdge->setVertex(1, this->GraphOptimizer.vertex(i));
      slamEdge->setMeasurement(relativeTransform);
      slamEdge->setInformation(covMatrix.inverse());
      this->GraphOptimizer.addEdge(slamEdge);
    }
  }

  // Handle GPS data
  int prevFoundId = -1;
  for (unsigned int i = 0; i < gpsPoses.size(); ++i)
  {
    // TODO can be optimized in order to not search again through all slam poses.
    int foundId = FindClosestSlamPose(gpsPoses[i], slamPoses);

    // Check matching validity, and ensure that the found slam pose is different
    // from the previous one (to prevent matching a single SLAM point to 2 
    // different GPS points).
    // CHECK if SLAM points are sparser than GPS, the first GPS/SLAM match may not be the best one.
    if ((foundId != -1) && (foundId != prevFoundId))
    {
      prevFoundId = foundId;

      // Get current GPS pose and covariance
      Eigen::Isometry3d gpsPose = gpsPoses[i].GetIsometry();
      Eigen::Map<Eigen::Matrix3d> covMatrix((double*) gpsCov[i].data());

      // Add GPS position as a vertex
      auto* gpsVertex = new g2o::VertexPointXYZ;
      gpsVertex->setId(++idCount);
      gpsVertex->setEstimate(gpsPose.translation());
      gpsVertex->setFixed(true);
      this->GraphOptimizer.addVertex(gpsVertex);

      // Add an edge between the temporal closest SLAM pose and GPS point
      auto* gpsEdge = new g2o::EdgeSE3PointXYZ;
      gpsEdge->setVertex(0, this->GraphOptimizer.vertex(foundId));
      gpsEdge->setVertex(1, this->GraphOptimizer.vertex(idCount));
      gpsEdge->setMeasurement(Eigen::Vector3d::Zero());  // We want to merge this SLAM point to this GPS point.
      gpsEdge->setInformation(covMatrix.inverse());
      gpsEdge->setParameterId(0, 0);  // Tell the edge to use the sensor/GPS calibration which is already defined. CHECK only translation is used?
      this->GraphOptimizer.addEdge(gpsEdge);
    }
  }
}