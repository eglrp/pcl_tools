#include <math.h>
#include <iostream>
#include <Eigen/Geometry>
#include <pcl/common/angles.h>

#define DEG2RAD(x) (x)* M_PI /180

Eigen::Quaterniond toQuaternion( double roll,double pitch, double yaw)
{
	Eigen::Quaterniond q;
        // Abbreviations for the various angular functions
	double cy = cos(yaw * 0.5);
	double sy = sin(yaw * 0.5);
	double cr = cos(roll * 0.5);
	double sr = sin(roll * 0.5);
	double cp = cos(pitch * 0.5);
	double sp = sin(pitch * 0.5);

	q.w() = cy * cr * cp + sy * sr * sp;
	q.x() = cy * sr * cp - sy * cr * sp;
	q.y() = cy * cr * sp + sy * sr * cp;
	q.z() = sy * cr * cp - cy * sr * sp;
	return q;
}
void toEulerAngle(const Eigen::Quaterniond& q, double& roll, double& pitch, double& yaw)
{
	// roll (x-axis rotation)
	double sinr = +2.0 * (q.w() * q.x() + q.y() * q.z());
	double cosr = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
	roll = atan2(sinr, cosr);

	// pitch (y-axis rotation)
	double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
	if (fabs(sinp) >= 1)
		pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
	else
		pitch = asin(sinp);

	// yaw (z-axis rotation)
	double siny = +2.0 * (q.w() * q.z() + q.x() * q.y());
	double cosy = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());  
	yaw = atan2(siny, cosy);
}


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

  q = toQuaternion(roll, pitch, yaw);
  std::cout << "quat = (" << q.w() << ", " << q.x() << "," << q.y() << ","<< q.z() << ")" <<std::endl;
  toEulerAngle(q,roll, pitch, yaw);
  std::cout << "roll, pitch, yaw " << pcl::rad2deg(roll) << ", " << pcl::rad2deg(pitch) << ", "  << pcl::rad2deg(yaw) << std::endl;

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

    //transfrome use Quaternion:
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
  

    //From Euler to Quaternion:
  Eigen::Quaterniond q3(Eigen::AngleAxisd(pcl::deg2rad((double)178.0), Eigen::Vector3d::UnitZ()));
  q3.normalize();
     //From Euler to Quaternion:
  Eigen::Quaterniond q4(Eigen::AngleAxisd(pcl::deg2rad((double)182.0), Eigen::Vector3d::UnitZ()));
  q4.normalize();

  q_diff = q3.inverse() * q4;
  euler = q_diff.toRotationMatrix().eulerAngles(2,1,0);
  std::cout << "q_diff euler  = \n"  << euler*57.3<< std::endl;

  Eigen::Quaterniond q5(Eigen::AngleAxisd(pcl::deg2rad((double)-172.0), Eigen::Vector3d::UnitZ()));
  q5.normalize();
  q_sum = q5 * q3;
  euler = q_sum.toRotationMatrix().eulerAngles(2,1,0);
  std::cout << "q_sum euler  = \n"  << euler*57.3<< std::endl;
  //q3.normalize();
  //euler = q3.toRotationMatrix().eulerAngles(2,1,0);
  //std::cout << "q3 euler  = \n"  << euler*57.3<< std::endl;
  
  Eigen::Quaterniond q6(1.0,0,0,0);

  euler = q6.toRotationMatrix().eulerAngles(2,1,0);
  std::cout << "q6 euler  = \n"  << euler*57.3<< std::endl;

}
