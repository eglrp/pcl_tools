//
// Created by robosense on 2018-1-4
//

/**
 * @file don_segmentation.cpp
 * Difference of Normals Example for PCL Segmentation Tutorials.
 *
 * @author Yani Ioannou
 * @date 2012-09-24
 */
#include <string>
#include <pcl/common/random.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>

#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/don.h>

#include <pcl/console/time.h>
#include <pcl/console/parse.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
using namespace std;


double threshold = 0.2;

void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s input_normal_small.pcd input_normal_small.pcd  <options> [optional_arguments]\n", argv[0]);
  print_info ("  where options are:\n");
  print_info ("                     -threshold X = threshold for DoN magnitude (default: ");
  print_value ("%f", threshold); print_info (")\n");
}

int
main (int argc, char *argv[])
{
  if (argc < 3)
  {
    printHelp (argc, argv);
    exit (EXIT_FAILURE);
  }
  parse_argument (argc, argv, "-threshold", threshold);
  pcl::PointCloud<PointNormal>::Ptr normals_small_scale (new pcl::PointCloud<PointNormal>);
  pcl::PointCloud<PointNormal>::Ptr normals_large_scale (new pcl::PointCloud<PointNormal>);

  if ( pcl::io::loadPCDFile <PointNormal> (argv[1], *normals_small_scale) == -1)
  {
    std::cout << "Cloud reading failed." << std::endl;
    return (-1);
  }
  if ( pcl::io::loadPCDFile <PointNormal> (argv[2], *normals_large_scale) == -1)
  {
    std::cout << "Cloud reading failed." << std::endl;
    return (-1);
  }

  // Create output cloud for DoN results
  PointCloud<PointNormal>::Ptr doncloud (new pcl::PointCloud<PointNormal>);
  copyPointCloud<PointNormal, PointNormal>(*normals_small_scale, *doncloud);

  cout << "Calculating DoN... " << endl;
  // Create DoN operator
  pcl::DifferenceOfNormalsEstimation<PointNormal, PointNormal, PointNormal> don;
  don.setInputCloud (normals_small_scale);
  don.setNormalScaleLarge (normals_large_scale);
  don.setNormalScaleSmall (normals_small_scale);
  if (!don.initCompute ())
  {
    std::cerr << "Error: Could not intialize DoN feature operator" << std::endl;
    exit (EXIT_FAILURE);
  }
  // Compute DoN
  don.computeFeature (*doncloud);

  // Save DoN features
  pcl::PCDWriter writer;
  writer.write<pcl::PointNormal> ("don.pcd", *doncloud, false);

  pcl::PointCloud<PointNormal>::Ptr doncloud_filtered (new pcl::PointCloud<PointNormal>),
  doncloud_remove (new pcl::PointCloud<PointNormal>);

  for(auto p : *doncloud)
  {
    if(p.curvature <=threshold)
    {
      doncloud_filtered->push_back(p);
    }
    else
    {
      doncloud_remove->push_back(p);
    }
  }


  std::cout << "Filtered Pointcloud: " << doncloud->points.size () << " data points." << std::endl;
  writer.write<pcl::PointNormal> ("don_filtered.pcd", *doncloud_filtered, false);
  writer.write<pcl::PointNormal> ("doncloud_remove.pcd", *doncloud_remove, false);
}