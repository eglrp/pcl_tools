/**
 * @file pca_estimation.cpp
 * PCA estimation.
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
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/pca.h>
#include <pcl/common/copy_point.h>
#include <pcl/common/file_io.h>
#include <pcl/common/centroid.h>
using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

double default_radius = 0.2;
bool  default_viewer = true;
bool  default_full = false;
double default_scale = 1;

typedef pcl::PointXYZ pType;

void printHelp (int, char **argv)
{
  print_error ("Syntax is: %s input.pcd <options> output.pcd [optional_arguments]\n", argv[0]);
  print_info ("  where options are:\n");
  print_info ("                     -radius X = use a radius of Xm around each point to determine the neighborhood (default: ");
  print_value ("%f", default_radius); print_info (")\n");
  print_info ("                     -full 0/1 = pca of all points (default: ");
  print_value ("%f", default_full); print_info (")\n");
  print_info ("                     -view 0/1      = show point cloud using PCLVisualizer  (default: ");
  print_value ("%d", default_viewer); print_info (")\n");
  print_info ("                     -scale X      = pcd vector scale  (default: ");
  print_value ("%f", default_scale); print_info (")\n");
}

int main(int argc, char **argv)
{
  print_info("xyz field pca estimation, use: %s -h\n", argv[0]);

  if(argc < 3)
  {
    printHelp(argc, argv);
    return (-1);
  }

  parse_argument (argc, argv, "-radius", default_radius);
  parse_argument (argc, argv, "-view", default_viewer);

  parse_argument (argc, argv, "-full", default_full);
  parse_argument (argc, argv, "-scale", default_scale);
  if(default_radius < 0.001)
  {
    print_error ("radius is too small.\n");
  }
  vector<int> p_file_indices;
  p_file_indices = parse_file_extension_argument (argc, argv, ".pcd");
  string f_output;
  if(!default_full)
  {
    if (p_file_indices.size () != 2)
    {
      print_error ("Need one input PCD file and one output.pcd to continue.\n");
      return (-1);
    }
  f_output = argv[p_file_indices[1]];
  }
  else{
    if (p_file_indices.size () < 1)
    {
      print_error ("Need one input PCD file  to continue.\n");
      return (-1);
    }
  }


  pcl::console::TicToc tic;

  // Load the first file
  Eigen::Vector4f translation;
  Eigen::Quaternionf orientation;
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2);

  if (io::loadPCDFile(argv[p_file_indices[0]],*cloud, translation, orientation) != 0)
    return (-1);

  print_info("load file %s finish\n", argv[p_file_indices[0]]);

  PointCloud<PointXYZ>::Ptr cloud_in (new PointCloud<PointXYZ>);

  fromPCLPointCloud2 (*cloud, *cloud_in);

  std::vector< int > index;
  removeNaNFromPointCloud(*cloud_in,*cloud_in,index);

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud (cloud_in);
  kdtree.setSortedResults(false);

  PointIndices::Ptr pointIdxRadiusSearch(new PointIndices);
  std::vector<float> pointRadiusSquaredDistance;

  pcl::PCA<pType> pca;
  pca.setInputCloud(cloud_in);
  Eigen::Matrix3f eigen_vectors;
  Eigen::Vector3f eigen_value;

  tic.tic();
  if(default_full)
  {
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("PCA Demo"));
    eigen_vectors = pca.getEigenVectors ();
    eigen_value = pca.getEigenValues()/cloud_in->size();
    tic.toc_print();
    std::cout << "eigen_vectors: \n" <<  eigen_vectors << std::endl;
    std::cout << "eigen_value: \n" <<  eigen_value << std::endl;

    viewer->addPointCloud(cloud_in,"cloud");
    pType p_origin,p_1, p_2, p_3 ;
    pcl::computeCentroid(*cloud_in,p_origin);
    if(eigen_value[0] > 10)
    {
      eigen_value[0] = 10;
    }
    if(eigen_value[1] < 0.2)
    {
      eigen_value[1] = 0.2;
    }
    if(eigen_value[2] < 0.1)
    {
      eigen_value[2] = 0.1;
    }
    p_1.getVector3fMap() = default_scale * eigen_value[0] * eigen_vectors.col(0) + p_origin.getVector3fMap() ;
    p_2.getVector3fMap() = default_scale * eigen_value[1] * eigen_vectors.col(1) + p_origin.getVector3fMap() ;
    p_3.getVector3fMap() = default_scale * eigen_value[2] * eigen_vectors.col(2) + p_origin.getVector3fMap() ;

    tic.toc_print();
    if(default_viewer)
    {
      viewer->addArrow(p_1, p_origin, 1.0, 0.0, 0.0, false, "a_x");
      viewer->addArrow(p_2, p_origin, 0.0, 1.0, 0.0, false, "a_y");
      viewer->addArrow(p_3, p_origin, 0.0, 0.0, 1.0, false, "a_z");
      //viewer->addCoordinateSystem(100,"cord");
      while (!viewer->wasStopped ())
      {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
      }
    }
  }//default_full true
  else
  {
    PointCloud<Normal> cloud_pca0,cloud_pca1,cloud_pca2;
    Normal p_normal;
    for(auto point : *cloud_in)
    {
      if(kdtree.radiusSearch(point, default_radius,pointIdxRadiusSearch->indices, pointRadiusSquaredDistance) > 3)
      {
        pca.setIndices(pointIdxRadiusSearch);
        eigen_vectors = pca.getEigenVectors ();
        eigen_value = pca.getEigenValues();

        p_normal.getNormalVector3fMap() = eigen_vectors.col(0);
        p_normal.curvature = eigen_value[0];
        cloud_pca0.push_back(p_normal);

        p_normal.getNormalVector3fMap() = eigen_vectors.col(1);
        p_normal.curvature = eigen_value[1];
        cloud_pca1.push_back(p_normal);

        p_normal.getNormalVector3fMap() = eigen_vectors.col(2);
        p_normal.curvature = eigen_value[2];
        cloud_pca2.push_back(p_normal);
      }
      else
      {
        p_normal.normal_x = 0;
        p_normal.normal_y = 0;
        p_normal.normal_z = 0;
        p_normal.curvature = 0;
        cloud_pca0.push_back(p_normal);
        cloud_pca1.push_back(p_normal);
        cloud_pca2.push_back(p_normal);
      }
    }
    pcl::PCLPointCloud2 output_normals ,output;
    PCDWriter w;
    string filename = getFilenameWithoutExtension(f_output) ;

    toPCLPointCloud2(*cloud_in,*cloud);
    toPCLPointCloud2 (cloud_pca0, output_normals);
    concatenateFields (*cloud, output_normals, output);
    w.writeBinaryCompressed (filename+"_pca0.pcd", output, translation, orientation);

    toPCLPointCloud2 (cloud_pca1, output_normals);
    concatenateFields (*cloud, output_normals, output);
    w.writeBinaryCompressed (filename+"_pca1.pcd", output, translation, orientation);

    toPCLPointCloud2 (cloud_pca2, output_normals);
    concatenateFields (*cloud, output_normals, output);
    w.writeBinaryCompressed (filename+"_pca2.pcd", output, translation, orientation);
    tic.toc_print();
  }//default_full false

  return (0);
}


