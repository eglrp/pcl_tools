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
  Eigen::Quaterniond q0(Eigen::AngleAxisd(pcl::deg2rad((double)0.0), Eigen::Vector3d::UnitZ()));
  q0.normalize();
  euler = q0.toRotationMatrix().eulerAngles(2,1,0);
  std::cout << "q0 euler  = \n"  << euler*57.3<< std::endl;

  //From Euler to Quaternion:
  Eigen::Quaterniond q1(Eigen::AngleAxisd(pcl::deg2rad((double)60.0), Eigen::Vector3d::UnitZ()));
  q1.normalize();
    //From Euler to Quaternion:
  Eigen::Quaterniond p = Eigen::Quaterniond(0, 1, 1, -3);  // 空间点（1,1，,3）的坐标变换 , w=0，纯四元素表示坐标
  Eigen::Quaterniond p_new = q1 * p * q1.inverse();
  std::cout << p_new.vec() << std::endl;
  std::cout << (q1.inverse() * p_new * q1).vec()<< std::endl;


  Eigen::Quaterniond q2(Eigen::AngleAxisd(pcl::deg2rad((double)70.0), Eigen::Vector3d::UnitZ()));
  q2.normalize();
  //std::cout << "q_d2" << std::endl << q_d2.coeffs() << std::endl;

  //angular difference
  Eigen::Quaterniond q_diff = q1.inverse() * q2;
  euler = q_diff.toRotationMatrix().eulerAngles(2,1,0);
  std::cout << "q_diff euler  = \n"  << euler*57.3<< std::endl;
 

 //angular sum
  Eigen::Quaterniond q_sum = q1 * q2;
  euler = q_sum.toRotationMatrix().eulerAngles(2,1,0);
  std::cout << "q_sum euler  = \n"  << euler*57.3<< std::endl;
  
  //q3.normalize();
  //euler = q3.toRotationMatrix().eulerAngles(2,1,0);
  //std::cout << "q3 euler  = \n"  << euler*57.3<< std::endl;
  


}
