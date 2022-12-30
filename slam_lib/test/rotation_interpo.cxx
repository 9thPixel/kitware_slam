
#include <LidarSlam/BasaltSpline.h>
#include <iostream>
#include <iomanip>

using namespace LidarSlam;

// Create poseStamped from a quaternion (w,x,y,z) and a time t
PoseStamped createPoseStamped(double x, double y, double z, double w, double t)
{
  Eigen::Quaterniond quat(w, x, y, z);
  // std::cout<< "quat : " << quat.coeffs().transpose() << std::endl;
  quat.normalize();
  // std::cout<< "quat : " << quat.coeffs().transpose() << "| t : " << t << std::endl;
  Eigen::UnalignedIsometry3d pose = Eigen::Translation3d::Identity() * quat;
  return (PoseStamped{pose, t});
}

//* PATH : /home/lea/goinfre/interpo_CamRoll
// Print interpolation into a csv with the format :
// X, Y, Z, Rx1, Ry1, Rz1, Rx2, Ry2, Rz2, Rx3, Ry3, Rz3, T
// NbrPt is the number of points by interval
void  print_csv(LidarSlam::Interpolation::IModel *interpoSpline, std::vector<PoseStamped>& vecData, unsigned int nbrPt,
                std::string name_csv, bool extrapolate = false)
{
  // Open file
  std::string path = "/home/abourbou/goinfre/interpo_CamRoll/" + name_csv;
  std::cout << "path : " << path << std::endl;
  std::ofstream file(path + ".csv", std::ofstream::out | std::ofstream::trunc);
  int i = 1;
  while (!file.is_open())
  {
    file.open(path + "(" + std::to_string(i) + ").csv");
    ++i;
    if (i >= 1000)
    {
      std::cout << "unable to open file" << std::endl;
      exit(2);
    }
  }

  file << "X,Y,Z,Rx1,Ry1,Rz1,Rx2,Ry2,Rz2,Rx3,Ry3,Rz3,T" << std::endl;
  if (!extrapolate)
  {
    // print Spline in a csv
    for (size_t i = 2; i <= vecData.size() - 2; ++i)
    {
      auto timeStep = (vecData[i + 1].Time - vecData[i].Time) / nbrPt;
      auto t = vecData[i].Time;
      for (t; t <= vecData[i + 1].Time; t+=timeStep)
      {
        auto iso  = (*interpoSpline)(t).matrix();
        // std::cout << "iso : " << std::endl << iso << std::endl;
        file << iso(0, 3) << "," << iso(1, 3) << "," << iso(2, 3) << "," 
             << iso(0, 0) << "," << iso(1, 0) << "," << iso(2, 0) << "," 
             << iso(0, 1) << "," << iso(1, 1) << "," << iso(2, 1) << "," 
             << iso(0, 2) << "," << iso(1, 2) << "," << iso(2, 2) << "," 
             << t << std::endl;
      }
    }
  }
  else
  {
    //* BEFORE
    {
    i = 0;
    auto timeStep = (vecData[i + 1].Time - vecData[i].Time) / nbrPt;
    auto t = vecData[i].Time;
    for (t = 2 * vecData[i].Time - vecData[i + 1].Time; t <= vecData[i + 1].Time + 0.0005; t+=timeStep)
    {
      auto iso  = (*interpoSpline)(t).matrix();
      // std::cout << "iso : " << std::endl << iso << std::endl;
      file << iso(0, 3) << "," << iso(1, 3) << "," << iso(2, 3) << "," 
           << iso(0, 0) << "," << iso(1, 0) << "," << iso(2, 0) << "," 
           << iso(0, 1) << "," << iso(1, 1) << "," << iso(2, 1) << "," 
           << iso(0, 2) << "," << iso(1, 2) << "," << iso(2, 2) << "," 
           << t << std::endl;
    }
    }
    //* AFTER
    i = vecData.size() - 2;
    auto timeStep = (vecData[i + 1].Time - vecData[i].Time) / nbrPt;
    auto t = vecData[i].Time;
    for (t; t <= 2 * vecData[i + 1].Time - vecData[i].Time; t+=timeStep)
    {
      auto iso  = (*interpoSpline)(t).matrix();
      // std::cout << "iso : " << std::endl << iso << std::endl;
      file << iso(0, 3) << "," << iso(1, 3) << "," << iso(2, 3) << "," 
           << iso(0, 0) << "," << iso(1, 0) << "," << iso(2, 0) << "," 
           << iso(0, 1) << "," << iso(1, 1) << "," << iso(2, 1) << "," 
           << iso(0, 2) << "," << iso(1, 2) << "," << iso(2, 2) << "," 
           << t << std::endl;
    }
  }
  file.close();
}

int main(void)
{
  std::vector<PoseStamped> vecPoseStamped;
  Interpolation::RotSpline interpoSpline(vecPoseStamped);

  // //* TEST quat strange
  // vecPoseStamped.clear();
  // vecPoseStamped.push_back(createPoseStamped(2.34, 3.5, 5.652321, 1, 0));
  // vecPoseStamped.push_back(createPoseStamped(-0.111, -0.45, 0.44445, 0, 1));
  // vecPoseStamped.push_back(createPoseStamped(0.89, 2.5, 0, 2.3, 2));
  // vecPoseStamped.push_back(createPoseStamped(-2.34, 3.00, 0.5652321, -1.678, 3));
  // interpoSpline = {vecPoseStamped};
  // print_csv(&interpoSpline, vecPoseStamped, 30, "strange_rotation_uniform");

  //* SIMPLE UNIFORM
  //                                          x       y     z     w    t
  vecPoseStamped.clear();
  double t = -1.;
  double step = (4. - -1.) / 4;
  int i = 0;
  vecPoseStamped.push_back(createPoseStamped( 0,      0,    0,    1,  t+step*i++));
  vecPoseStamped.push_back(createPoseStamped( 0,      0,    1,    1,  t+step*i++));
  vecPoseStamped.push_back(createPoseStamped( 0,      0,    -1,   1,  t+step*i++));
  vecPoseStamped.push_back(createPoseStamped( 0,      0,    -1,   0,  t+step*i++));
  vecPoseStamped.push_back(createPoseStamped(-0.5,  0.5,    -1,   0,  t+step*i));
  interpoSpline = {vecPoseStamped};
  print_csv(&interpoSpline, vecPoseStamped, 50, "simple_rotation_uniform");

  // //* SIMPLE NON UNIFORM
  // //                                          x       y     z     w    t
  // vecPoseStamped.clear();
  // vecPoseStamped.push_back(createPoseStamped( 0,      0,    0,    1,  -1));
  // vecPoseStamped.push_back(createPoseStamped( 0,      0,    1,    1,  1));
  // vecPoseStamped.push_back(createPoseStamped( 0,      0,    -1,   1,  1.5));
  // vecPoseStamped.push_back(createPoseStamped( 0,      0,    -1,   0,  3));
  // vecPoseStamped.push_back(createPoseStamped(-0.5,  0.5,    -1,   0,  4));
  // interpoSpline = {vecPoseStamped};
  // print_csv(&interpoSpline, vecPoseStamped, 50, "simple_rotation_non_uniform");

  // //* MEDIUM NON UNIFORM
  // //                                          x       y     z     w    t
  // vecPoseStamped.clear();
  // vecPoseStamped.push_back(createPoseStamped( 0,      0,    0,    1,  0));
  // vecPoseStamped.push_back(createPoseStamped( 0,      0,    1,    0,  1));
  // vecPoseStamped.push_back(createPoseStamped( 0,      0,    -1,   0,  1.5));
  // vecPoseStamped.push_back(createPoseStamped( 1,      1,    0,   0,  4));
  // interpoSpline = {vecPoseStamped};
  // print_csv(&interpoSpline, vecPoseStamped, 30, "medium_rotation_non_uniform");

  // // //* SIMPLE EXTRAPOLATION UNIFORM
  // vecPoseStamped.clear();
  // vecPoseStamped.push_back(createPoseStamped( 0,      0,    0,    1,  0));
  // vecPoseStamped.push_back(createPoseStamped( 0,      0,    1,    0,  1));
  // vecPoseStamped.push_back(createPoseStamped( 0,      0,    -1,   0,  3));
  // vecPoseStamped.push_back(createPoseStamped( 1,      1,    0,   0,  4));
  // interpoSpline = {vecPoseStamped};
  // print_csv(&interpoSpline, vecPoseStamped, 30, "extrapolation_rotation_non_uniform", true);

}