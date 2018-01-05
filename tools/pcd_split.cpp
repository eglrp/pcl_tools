#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/filters/voxel_grid.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;


void printHelp (int, char **argv)
{
  print_error ("Syntax is: %s input.pcd n \n", argv[0]);
}

int main(int argc, char **argv)
{
  print_info("分割点云, use: %s -h\n", argv[0]);

  if(argc != 3)
  {
    printHelp(argc, argv);
    return (-1);
  }

  // Load the first file
  pcl::PointCloud<pcl::PointXYZ> cloud_in,cloud_out;
  if (pcl::io::loadPCDFile(argv[1],cloud_in) != 0)
  {
    print_error ("load PCD file error.\n");
    return (-1);
  }
  print_info("load file %s finish\n", argv[1]);
  int splitn = atoi(argv[2]);
  int size = ceil(cloud_in.size()/splitn);
  int fname_index = 1;
  cloud_out.reserve(splitn *1.1);
  for(int j = 0; j < cloud_in.size(); ++j)
  {
    cloud_out.push_back(cloud_in.at(j));
    if (cloud_out.size() >=size)
    {
      std::string fname = std::to_string(fname_index) +".pcd";
      pcl::io::savePCDFileBinary(fname, cloud_out);
      print_info("save file %s finish\n", fname.c_str());
      fname_index++;
      cloud_out.clear();
    }
  }
  std::cout << "point cloud sort finish "<< std::endl;
}


