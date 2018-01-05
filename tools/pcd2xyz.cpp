//
// Created by g on 2018-01-05.
// @TODO 文件加锁
//

#include <iostream>
#include <vector>
#include <fstream>
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
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/transforms.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
using namespace std;

int    default_psize = 0;
int    default_ptype = 0;
void printHelp (int, char **argv)
{
  print_error ("Syntax is: %s input.pcd output.xyz <options> [optional_arguments] \n", argv[0]);
  print_info ("  where options are:\n");
  print_info ("                     -psize X =  points size to save (default: ");
  print_value ("%d", default_psize); print_info (")\n");
  print_info ("                     -ptype X =  point type\n   ");
  print_info ("                                 0 (PointXYZ)\n");
  print_info ("                                 1 (PointXYZI)\n ");
  print_info ("                                 2 (PointXYZRGBA)\n");
  print_info ("                                 3 (PointNormal)\n");
  print_info ("                                 (default: ");
  print_value ("%d", default_ptype); print_info (")\n");
}

int main(int argc, char **argv)
{
  print_info("pcd转xyz格式, use: %s -h\n", argv[0]);

  if(argc < 3)
  {
    printHelp(argc, argv);
    return (-1);
  }
  parse_argument (argc, argv, "-psize", default_psize);
  parse_argument (argc, argv, "-ptype", default_ptype);

  vector<int> p_file_indices;
  p_file_indices = parse_file_extension_argument (argc, argv, ".pcd");

  if (p_file_indices.size () != 1)
  {
    print_error ("Need one input PCD file to continue.\n");
    return (-1);
  }
  // Load the first file
  pcl::console::TicToc tic;
  // Load the first file
  Eigen::Vector4f origin;
  Eigen::Quaternionf orientation;
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2);

  if (io::loadPCDFile(argv[p_file_indices[0]],*cloud, origin, orientation) != 0)
    return (-1);

  Eigen::Affine3f translation = Eigen::Affine3f::Identity();
  translation.prerotate(orientation);
  translation(0,3) = origin[0];
  translation(1,3) = origin[1];
  translation(2,3) = origin[2];

  print_info("load file %s finish, %d points\n", argv[1],cloud->data.size());

  std::string fname_save = argv[2];

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_xyzi(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normal(new pcl::PointCloud<pcl::PointNormal>());
  ofstream fout;
  fout.open(fname_save, ios::out|ios::trunc) ;
  int counter = 0;
  if  (!fout.is_open())
  {
    print_error("canot open file");
    return -1;
  }
  fout.setf(ios::fixed, ios::floatfield);
  fout.precision(4);  // 设置精度3位小数 默认国际标准单位，对于xyz，精度0.1mm
  switch(default_ptype)
  {
    case 0:
    {
      fromPCLPointCloud2 (*cloud, *cloud_xyz);
      pcl::transformPointCloud(*cloud_xyz,*cloud_xyz,translation);
      for (auto p : *cloud_xyz)
      {

        fout << p.x <<"  "<< p.y <<"  "<< p.z  << std::endl;
        counter++;
        if(counter % 1000000 == 0)
        {
          std::cout << counter << "/" << cloud_xyz->size()<< std::endl;
        }
        if(default_psize > 0 && counter >= default_psize)
        {
          break;
        }
      }
    }
      break;
    case 1:
    {
      fromPCLPointCloud2 (*cloud, *cloud_xyzi);
      pcl::transformPointCloud(*cloud_xyzi,*cloud_xyzi,translation);
      for (auto p : *cloud_xyzi)
      {
        fout << p.x <<"  "<< p.y <<"  "<< p.z <<"  "<< p.intensity << std::endl;
        counter++;
        if(counter % 1000000 == 0)
        {
          std::cout << counter << "/" << cloud_xyzi->size()<< std::endl;
        }
        if(default_psize > 0 && counter >= default_psize)
        {
          break;
        }
      }
    }
      break;

    case 2:
    {
      fromPCLPointCloud2 (*cloud, *cloud_rgb);
      pcl::transformPointCloud(*cloud_rgb,*cloud_rgb,translation);
      for (auto p : *cloud_rgb)
      {
        fout << p.x <<"  "<< p.y <<"  "<< p.z <<"  "<< "  " << (short)p.r  <<"  "<< (short)p.g <<"  " << (short)p.b<< std::endl;
        counter++;
        if(counter % 1000000 == 0)
        {
          std::cout << counter << "/" << cloud_rgb->size()<< std::endl;
        }
        if(default_psize > 0 && counter >= default_psize)
        {
          break;
        }
      }
    }
      break;
    case 3:
    {
      fromPCLPointCloud2 (*cloud, *cloud_normal);
      pcl::transformPointCloudWithNormals(*cloud_normal,*cloud_normal,translation);
      for (auto p : *cloud_normal)
      {
        fout << p.x <<"  "<< p.y <<"  "<< p.z <<"  "<< "  " << p.normal_x  <<"  "<< p.normal_y <<"  " << p.normal_z<<"  " << p.curvature<< std::endl;
        counter++;
        if(counter % 1000000 == 0)
        {
          std::cout << counter << "/" << cloud_normal->size()<< std::endl;
        }
        if(default_psize > 0 && counter >= default_psize)
        {
          break;
        }
      }
    }
      break;
  }
  fout.close();
  print_info("save file %s finish\n", argv[2]);
}
