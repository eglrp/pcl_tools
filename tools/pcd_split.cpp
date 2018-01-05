/**
 * @file pcd_split.cpp
 *
 * @author GuoLeiming
 * @date 2018-1-4
 */
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
#include <pcl/common/file_io.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
using namespace std;

int psize = 1000000; //1 Million
int fnum = 0; //默认使用点云个数分割
int ftype = 1;

void printHelp (int, char **argv)
{
  print_error ("Syntax is: %s input.pcd output.pcd <options> [optional_arguments] \n", argv[0]);
  print_info ("  where options are:\n");
  print_info ("                     -psize X =  max points size of one file to save (default: ");
  print_value("%d", psize); print_info (")\n");
  print_info ("                     -fnum X =  files number\n   ");
  print_info ("                                 (default: ");
  print_value("%d", fnum); print_info (")\n");
  print_info ("                     -ftype X =  point type\n   ");
  print_info ("                                 0 (ascii)\n");
  print_info ("                                 1 (binary)\n ");
  print_info ("                                 2 (compress binary)\n");
  print_info ("                                 (default: ");
  print_value ("%d", ftype); print_info (")\n");
}

int main(int argc, char **argv)
{
  print_info("split pcd file , use: %s -h\n", argv[0]);

  if(argc < 3)
  {
    printHelp(argc, argv);
    return (-1);
  }

  parse_argument (argc, argv, "-psize", psize);
  parse_argument (argc, argv, "-fnum", fnum);
  parse_argument (argc, argv, "-ftype", ftype);

  vector<int> p_file_indices;
  p_file_indices = parse_file_extension_argument (argc, argv, ".pcd");

  if (p_file_indices.size () != 2)
  {
    print_error ("Need one input PCD file and output.pcd to continue.\n");
    return (-1);
  }
  // Load the first file
  pcl::console::TicToc tic;
  // Load the first file
  Eigen::Vector4f origin;
  Eigen::Quaternionf orientation;
  pcl::PCLPointCloud2::Ptr cloud_in (new pcl::PCLPointCloud2);

  if (io::loadPCDFile(argv[p_file_indices[0]],*cloud_in, origin, orientation) != 0)
    return (-1);
  int cloud_in_size = cloud_in->width * cloud_in->height;
  print_info("load file %s, point size %d\n", argv[1],cloud_in_size);
  string fname = getFilenameWithoutExtension(argv[p_file_indices[1]]);
/*  Eigen::Affine3f translation = Eigen::Affine3f::Identity();
  translation.prerotate(orientation);
  translation(0,3) = origin[0];
  translation(1,3) = origin[1];
  translation(2,3) = origin[2];*/
  PCDWriter writer;
  if(fnum > 0)
  {
    psize = ceil(cloud_in_size/fnum);
  }
  else
  {
    if(psize >= cloud_in_size)
    {
      print_error ("psize is larger than input file points size.\n");
      return -1;
    }
    fnum = cloud_in_size/psize + 1;
  }

  int fname_suffix = 0;

  for(int i = 0 ; i < fnum; i++)
  {
    string name = fname +"_"+to_string(fname_suffix)+".pcd";
    pcl::PCLPointCloud2 cloud_out;
    vector<int> indices;
    for(int j = 0; j < psize; ++j)
    {
      int index = i * psize + j;
      if(index < cloud_in_size)
      {
        indices.push_back(index);
      }
    }
    pcl::copyPointCloud(*cloud_in,indices,cloud_out);
    writer.writeBinary(name,cloud_out,origin,orientation);
    cout << "save file " << name <<std::endl;
    fname_suffix++;
  }
  tic.toc_print();
}


