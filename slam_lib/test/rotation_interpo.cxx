
#include <LidarSlam/BasaltSpline.h>

using namespace LidarSlam;

int main(void)
{
  std::cout << "Hello world!" << std::endl;
  Sophus::SE3<double> test_sophus;
  Interpolation::RotSpline test_spline(std::vector<PoseStamped>());
}