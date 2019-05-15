#include <ros/ros.h>
#include <iostream>
#include <vector>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/console/time.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/PointIndices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/surface/convex_hull.h>


typedef pcl::PointXYZ PointType;

ros::Publisher pub1, pub2, pub3, pub4;

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)  {


  sensor_msgs::PointCloud2::Ptr output (new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2::Ptr output_vg (new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2::Ptr output_plane (new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2::Ptr output_pass (new sensor_msgs::PointCloud2);

// Container for original & filtered data
  pcl::PointCloud<PointType>::Ptr cloud (new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr cloud_vg (new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr cloud_pass (new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr cloud_plane (new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr cloud_f (new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr clustered_cloud (new pcl::PointCloud<PointType>);


// Convert to PCL data type
  pcl::fromROSMsg(*input, *cloud);

//std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl;



// Voxel Grid filter
 pcl::VoxelGrid<PointType> vg;
  vg.setInputCloud (cloud);
  vg.setLeafSize (0.01f, 0.01f, 0.01f);
  vg.filter (*cloud_vg);


// Pass Through filter
  pcl::PassThrough<PointType> pass;
  pass.setInputCloud (cloud_vg);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.5);
  //pass.setFilterFieldName ("x");
 // pass.setFilterLimits (0.0, 1);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_pass);

  pcl::toROSMsg(*cloud_pass, *output_pass);

  output_pass->header.frame_id = "/camera_depth_optical_frame";
  pub4.publish (output_pass);

/* //Statistical Outlier Removal filter
  pcl::StatisticalOutlierRemoval<PointType> sor;
  sor.setInputCloud (cloud_pass);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.setNegative (false);
  sor.filter (*cloud_sor);*/

// SAC SEGMENTATION
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
// Create the segmentation object
  pcl::SACSegmentation<PointType> seg;
// Optional
  seg.setOptimizeCoefficients (true);
// Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.01);

  int i=0, nr_points = (int) cloud_pass->points.size ();

  //std::cout << "Nr of points in cloud_pass: " << nr_points <<std::endl;

  while (cloud_pass->points.size () > 0.3 * nr_points)
  {
    seg.setInputCloud (cloud_pass);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

 // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<PointType> extract;
    extract.setInputCloud (cloud_pass);
    extract.setIndices (inliers);
   //outliers
    extract.setNegative (false);
    extract.filter (*cloud_plane);
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud_pass = *cloud_f;
   }


  pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
  tree->setInputCloud (cloud_pass);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointType> ec;
  ec.setClusterTolerance (0.02); // 1 cm
  ec.setMinClusterSize (80);
  ec.setMaxClusterSize (250);
  ec.setSearchMethod (tree);

  ec.setInputCloud (cloud_pass);
  ec.extract (cluster_indices);

  int j=0;
  std::vector<int>::const_iterator pit;
  std::vector<pcl::PointIndices>::const_iterator it;

 for (it = cluster_indices.begin (); it != cluster_indices.end (); ++it) {
    pcl::PointCloud<PointType>::Ptr cloud_cluster (new pcl::PointCloud<PointType>);
    for (pit = it->indices.begin (); pit != it->indices.end (); ++pit) {
       //push_back: add a point to the end of the existing vector
       cloud_cluster->points.push_back (cloud_pass->points[*pit]);

    }
    *clustered_cloud += *cloud_cluster;
    std::cout << "Nr of points in cloud_cluster: " << cloud_cluster->points.size () <<std::endl;


 }

//SAVING pcd file...
  if (clustered_cloud->points.size()> 80) {
        pcl::io::savePCDFileASCII("referenciamodell.pcd",*clustered_cloud);
        std::cout<< "Saved " << clustered_cloud->points.size() <<" data points" << std::endl;
    }

  pcl::toROSMsg(*cloud_vg, *output_vg);
  pcl::toROSMsg(*clustered_cloud, *output);
  pcl::toROSMsg(*cloud_plane, *output_plane);


  output_vg->header.frame_id = "/camera_depth_optical_frame";
  output->header.frame_id = "/camera_depth_optical_frame";
  output_plane->header.frame_id = "/camera_depth_optical_frame";


  output->header.stamp=ros::Time::now();
  // Publish the data.
  pub1.publish (output_vg);
  pub2.publish (*output);
  pub3.publish (output_plane);

}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "RTobject_detection");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub1 = nh.advertise<sensor_msgs::PointCloud2> ("output_vg", 1);
  pub2 = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
  pub3 = nh.advertise<sensor_msgs::PointCloud2> ("output_plane", 1);
  pub4 = nh.advertise<sensor_msgs::PointCloud2> ("output_pass", 1);


  // Spin
  ros::spin ();
}

