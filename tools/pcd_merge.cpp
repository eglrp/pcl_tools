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
#include <pcl/registration/transforms.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/file_io.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;


void printHelp (int, char **argv)
{
  print_error ("Syntax is: %s dir_in out.pcd dir_out\n", argv[0]);
}

int main(int argc, char **argv)
{
  print_info("合并文件夹下所有pcd文件, use: %s -h\n", argv[0]);

  if(argc != 3)
  {
    printHelp(argc, argv);
    return (-1);
  }

  std::string dir = argv[1];
  std::vector<std::string> file_names ;
  pcl::getAllPcdFilesInDirectory(dir,file_names);
  std::vector<std::vector<std::string>> sub_files_index;
  if(file_names.size() <1)
  {
    print_error("no pcd files");
    return -1;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ> cloud_filtered,cloud_out;

  //cloud_in.reserve(10000000);
  cloud_out.reserve(150000000);
  for(auto f : file_names)
  {
    f = dir+ "/" +f;
    pcl::io::loadPCDFile(f,*cloud_in);
    Eigen::Affine3f trans = Eigen::Affine3f::Identity();
    trans.prerotate(cloud_in->sensor_orientation_);
    trans(0,3) = cloud_in->sensor_origin_[0];
    trans(1,3) = cloud_in->sensor_origin_[1];
    trans(2,3) = cloud_in->sensor_origin_[2];
    pcl::transformPointCloud(*cloud_in,*cloud_in,trans);
    std::cout << "load " << f << " " <<cloud_in->size() << " points" <<std::endl;
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud_in);
    sor.setLeafSize (0.05f, 0.05f, 0.05f);
    sor.filter (cloud_filtered);

    cloud_out += cloud_filtered;
  }
  pcl::io::savePCDFileBinary(argv[2], cloud_out);
  std::cout << "save pcd file "<< argv[2] << std::endl;
}


