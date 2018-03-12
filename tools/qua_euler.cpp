#include <math.h>
#include <iostream>
#include <Eigen/Geometry>
#include <pcl/common/angles.h>

#define DEG2RAD(x) (x)* M_PI /180
int main(int argc, char **argv)
{
  // euler to matrix
  double x = 1, y = 2, z = 3, roll = DEG2RAD(30), pitch = DEG2RAD(45), yaw = DEG2RAD(60);
  Eigen::Affine3d trans = Eigen::Affine3d::Identity();
  trans.prerotate(Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
  trans.prerotate(Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()));
  trans.prerotate(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
  trans.pretranslate(Eigen::Vector3d(x,y,z));
  std::cout << trans.matrix() << std::endl;

  // matrix to euler
  Eigen::Vector3d euler = trans.rotation().eulerAngles(2,1,0);  // yaw pitch roll
  yaw = euler[0]; pitch = euler[1]; roll = euler[2];
  std::cout << "euler  = "  << euler*57.3<< std::endl;
  
  // matrix to quaternation
  Eigen::Quaterniond q(trans.rotation());
  q.normalize();
  std::cout << "quat = (" << q.w() << ", " << q.x() << "," << q.y() << ","<< q.z() << ")" <<std::endl;
  std::cout << "quat to RotationMatrix \n" << q.toRotationMatrix() << "\n"<< std::endl;

  trans = Eigen::Affine3d::Identity();
  trans.prerotate(Eigen::AngleAxisd(DEG2RAD(60), Eigen::Vector3d::UnitZ()));
  std::cout << trans.matrix() << std::endl;
  euler = trans.rotation().eulerAngles(2,1,0);  // yaw pitch roll
  std::cout << "pose  = "  << euler*57.3<< std::endl;

  //From Euler to Quaternion:
  q = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
      * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
  std::cout << "Quaternion" << std::endl << q.coeffs() << std::endl;
  
  


}
