#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

int main()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>); 
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  pcl::io::loadPCDFile ("newkube.pcd", *cloud);

  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);

  // Output datasets
  

  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch (0.01);

  // Compute the features
  ne.compute (*cloud_normals);


  pcl::concatenateFields(*cloud, *cloud_normals, *cloud_with_normals); 

  pcl::io::savePCDFileASCII ("scanned_model.pcd", *cloud_with_normals);
  
  


  
 
 
 
 
 
 
 
 
 
 
 
 
 
  //... visualize cloud
  pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
  viewer.showCloud (cloud);
  while (!viewer.wasStopped ())
  {
  }


  
    // visualize normals
  /* pcl::visualization::PCLVisualizer viewer("PCL Viewer");
  viewer.setBackgroundColor (0.0, 0.0, 0.5);
  viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, cloud_normals);
  
  while (!viewer.wasStopped ())
  {
    viewer.spinOnce ();
  }
  return 0; */
  



  
  
  /*
  // Initialization part
  dst.width = src.width;
  dst.height = src.height;
  dst.is_dense = true;
  dst.points.resize(dst.width * dst.height);

  // Assignment part
  for (int i = 0; i < cloud_normals->points.size(); i++)
  {
      dst.points[i].x = src.points[i].x;
      dst.points[i].y = src.points[i].y;
      dst.points[i].z = src.points[i].z;

      dst.points[i].r = src.points[i].r;
      dst.points[i].g = src.points[i].g;
      dst.points[i].b = src.points[i].b;

    // cloud_normals -> Which you have already have; generated using pcl example code 

      dst.points[i].curvature = cloud_normals->points[i].curvature;

      dst.points[i].normal_x = cloud_normals->points[i].normal_x;
      dst.points[i].normal_y = cloud_normals->points[i].normal_y;
      dst.points[i].normal_z = cloud_normals->points[i].normal_z;


      
  }
*/



  // cloud_normals.size () //should have the same size as the input cloud->size ()*
}