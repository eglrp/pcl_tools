#include <iostream>
#include <fstream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <liblas/liblas.hpp>
#include <vector>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <fstream>
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
using namespace std;

void printHelp (int, char **argv)
{
  print_error ("Syntax is: %s input.txt  out.pcd binary(1) \n", argv[0]);
}

struct  Pose
{
  float x;
  float y;
  float z;
  float roll;
  float pitch;
  float yaw;
};


int main(int argc, char** argv)
{
  if (argc < 4)
  {
    printHelp (argc, argv);
    return (-1);
  }

  pcl::PointCloud<pcl::PointXYZ> clout_out;
  std::ifstream ifs;
  std::string line;
  ifs.open(argv[1]);
  getline(ifs,line);

  while(getline(ifs,line))
  {
    std::string temp;
    std::stringstream sstream(line);

    Pose pose;
    sstream >> temp >>pose.x >>pose.y >> pose.z >> pose.roll >> pose.yaw;
    pcl::PointXYZ point;
    point.x = pose.x;
    point.y = pose.y;
    point.z = pose.z;
    clout_out.push_back(point);
  }
  ifs.close();
  char b = *argv[3];
  if (argc == 4 && b == '1')
  {
    pcl::io::savePCDFileBinary(argv[2],clout_out);
  }
  else
  {
    pcl::io::savePCDFileASCII(argv[2],clout_out);
  }

  print_info("save file %s finish\n", argv[2]);
  return (0);
}