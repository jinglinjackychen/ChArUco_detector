#include <fstream>
#include <boost/filesystem.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h> // pcl::ModelCoefficients
#include <pcl/io/pcd_io.h> // pcl::io::savePCDFileASCII, pcl::io::savePCDFileBinary
#include <pcl/point_types.h> // pcl::PointXYZ
#include <pcl/filters/passthrough.h> // pcl::passThrough
#include <pcl/common/transforms.h> // pcl::transformPointCloud
#include <pcl/sample_consensus/method_types.h> // pcl::SAC_RANSAC
#include <pcl/sample_consensus/model_types.h> // pcl::SACMODEL_PLANE
#include <pcl/segmentation/sac_segmentation.h> // pcl::SACSegmentation
#include <pcl/registration/icp.h> // pcl::IterativeClosestPoint
#include <pcl/filters/voxel_grid.h> // pcl::VoxelGrid
#include <visual_system/helper.h>
// message filters
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
// SRV
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <visual_system/get_xyz.h>
#include <visual_system/get_pc.h>
#include <visual_system/pc_is_empty.h>
#include <visual_system/check_valid.h>
#include <visual_system/get_surface_feature.h>
// MSG
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <visualization_msgs/Marker.h>
// Includes listed here

ros::Publisher _pub;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in; 
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcdtarget(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcdsource(new pcl::PointCloud<pcl::PointXYZRGB>);

void fixed_frame_cb(const sensor_msgs::PointCloud2ConstPtr& pc2_msg) {
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg ( *pc2_msg, *cloud_in);
}

void camera_frame_cb(const sensor_msgs::PointCloud2ConstPtr& next_pc2_msg) {
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2_in(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg ( *next_pc2_msg, *cloud2_in);

  // Perform ICP
  pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
  icp.setInputSource(cloud2_in);
  icp.setInputTarget(cloud_in);
  pcl::PointCloud<pcl::PointXYZRGB> Final;
  icp.align(Final);

  // Convert the pcl/PointCloud to sensor_msgs/PointCloud2
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg( *cloud2_in, output);
  // Publish the results
  _pub.publish( output );
}

int main (int argc, char** argv) {
  // Initialize ROS
  ros::init (argc, argv, "ICP");
  ros::NodeHandle n;
  pcl::io::loadPCDFile<pcl::PointXYZRGB>("/home/jacky/catkin_ws/rosbag/icp/2/1574151247830836.pcd", *pcdsource);
  pcl::io::loadPCDFile<pcl::PointXYZRGB>("/home/jacky/catkin_ws/rosbag/icp/1/1574148168244146.pcd", *pcdtarget);

  // Create ROS subscriber for fixed_frame topic
  //ros::Subscriber sub = n.subscribe("/pixel_to_xyz02/point_cloud",1,fixed_frame_cb);

  // Create ROS subscriber for camera_frame topic
  //ros::Subscriber sub2 = n.subscribe("/pixel_to_xyz04/point_cloud",1,camera_frame_cb);

  // Create ROS publisher for transformed pointcloud
  _pub = n.advertise<sensor_msgs::PointCloud2>("/iml_transform_point_cloud/transformed_point_cloud",1);

  pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
  icp.setInputSource(pcdsource);
  icp.setInputTarget(pcdtarget);
  pcl::PointCloud<pcl::PointXYZRGB> Final;
  icp.align(Final);
  printf("ICP fitness score: [%f] ",icp.getFitnessScore());

  pcl::io::savePCDFileASCII("final1.pcd",Final);

  // Spin
  ros::spin();
  return 0;
}
