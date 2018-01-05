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

using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

double default_radius = 0.2;
bool  default_viewer = false;
bool  default_full = false;

typedef pcl::PointXYZ pType;

void printHelp (int, char **argv)
{
  print_error ("Syntax is: %s input.pcd <options> output.pcd [optional_arguments]\n", argv[0]);
  print_info ("  where options are:\n");
  print_info ("                     -radius X = use a radius of Xm around each point to determine the neighborhood (default: ");
  print_value ("%f", default_radius); print_info (")\n");
  print_info ("                     -full 0/1 = pca of the all points (default: ");
  print_value ("%f", default_full); print_info (")\n");
  print_info ("                     -view 0/1      = show point cloud using PCLVisualizer  (default: ");
  print_value ("%d", default_viewer); print_info (")\n");
}

int main(int argc, char **argv)
{
  print_info("pcd主成分分析, use: %s -h\n", argv[0]);

  if(argc != 2)
  {
    printHelp(argc, argv);
    return (-1);
  }

  parse_argument (argc, argv, "-radius", default_radius);
  parse_argument (argc, argv, "-view", default_viewer);
  parse_argument (argc, argv, "-full", default_full);
  vector<int> p_file_indices;
  p_file_indices = parse_file_extension_argument (argc, argv, ".pcd");

  if (p_file_indices.size () != 2)
  {
    print_error ("Need one input PCD file and one output.pcd to continue.\n");
    return (-1);
  }

  pcl::console::TicToc tic;

  // Load the first file
  Eigen::Vector4f translation;
  Eigen::Quaternionf orientation;
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2);

  if (io::loadPCDFile(argv[p_file_indices[0]],*cloud, translation, orientation) != 0)
    return (-1);
  string f_output = argv[p_file_indices[1]];
  print_info("load file %s finish\n", argv[1]);

  PointCloud<PointXYZ>::Ptr cloud_in (new PointCloud<PointXYZ>);
  cloud.reset();
  fromPCLPointCloud2 (*cloud, *cloud_in);
  std::vector< int > index;
  removeNaNFromPointCloud(*cloud_in,*cloud_in,index);

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud (cloud_in);
  kdtree.setSortedResults(true);

  PointIndices::Ptr pointIdxRadiusSearch(new PointIndices);
  std::vector<float> pointRadiusSquaredDistance;


  pcl::PCA<pType> pca;
  pca.setInputCloud(cloud_in);
  Eigen::Matrix3f eigen_vectors;
  Eigen::Vector3f eigen_value;
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("PCA Demo"));
  tic.tic();
  if(default_full)
  {
    eigen_vectors = pca.getEigenVectors ();
    eigen_value = pca.getEigenValues();
    tic.toc_print();
    std::cout << "eigen_vectors " <<  eigen_vectors << std::endl;
    std::cout << "eigen_value " <<  eigen_value << std::endl;


    viewer->addPointCloud(cloud_in,"cloud");
    pType p_origin , p_1, p_2, p_3;
    p_origin.x = 0;
    p_origin.y = 0;
    p_origin.z = 0;

    p_1.x =  1e2 * eigen_vectors(0,0);
    p_1.y =  1e2 * eigen_vectors(1,0);
    p_1.z =  1e2 * eigen_vectors(2,0);

    p_2.x =  1e2 * eigen_vectors(0,1);
    p_2.y =  1e2 * eigen_vectors(1,1);
    p_2.z =  1e2 * eigen_vectors(2,1);

    p_3.x =  1e2 * eigen_vectors(0,2);
    p_3.y =  1e2 * eigen_vectors(1,2);
    p_3.z =  1e2 * eigen_vectors(2,2);

    tic.toc_print();
    if(default_viewer)
    {
      viewer->addArrow(p_1, p_origin, 1.0,0.0,0.0, false, "a_x");
      viewer->addArrow(p_2, p_origin, 0.0,1.0,0.0, false, "a_y");
      viewer->addArrow(p_3, p_origin, 0.0,0.0,1.0, false, "a_z");
      viewer->addCoordinateSystem(100,"cord");
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
        p_normal.normal_x = eigen_vectors(0,0);
        p_normal.normal_y = eigen_vectors(1,0);
        p_normal.normal_z = eigen_vectors(2,0);
        p_normal.curvature = eigen_value[0];
        cloud_pca0.push_back(p_normal);

        p_normal.normal_x = eigen_vectors(0,1);
        p_normal.normal_y = eigen_vectors(1,1);
        p_normal.normal_z = eigen_vectors(2,1);
        p_normal.curvature = eigen_value[1];
        cloud_pca1.push_back(p_normal);

        p_normal.normal_x = eigen_vectors(0,2);
        p_normal.normal_y = eigen_vectors(1,2);
        p_normal.normal_z = eigen_vectors(2,2);
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

  }//default_full false

  // Convert data back


  return (0);
}


