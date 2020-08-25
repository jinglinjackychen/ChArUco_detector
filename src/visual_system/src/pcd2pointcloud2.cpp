#include <vtkAutoInit.h>         
VTK_MODULE_INIT(vtkRenderingOpenGL);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);

#include <fstream>
#include <boost/filesystem.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Core>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h> // pcl::ModelCoefficients
#include <pcl/io/pcd_io.h> // pcl::io::savePCDFileASCII, pcl::io::savePCDFileBinary
#include <pcl/point_cloud.h>
#include <pcl/point_types.h> // pcl::PointXYZ
#include <pcl/filters/passthrough.h> // pcl::passThrough
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h> // pcl::transformPointCloud
#include <pcl/common/common.h>
#include <pcl/sample_consensus/method_types.h> // pcl::SAC_RANSAC
#include <pcl/sample_consensus/model_types.h> // pcl::SACMODEL_PLANE
#include <pcl/segmentation/sac_segmentation.h> // pcl::SACSegmentation
#include <pcl/registration/icp.h> // pcl::IterativeClosestPoint
#include <pcl/filters/voxel_grid.h> // pcl::VoxelGrid
#include <pcl/visualization/pcl_visualizer.h>
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
pcl::PointCloud<pcl::PointXYZRGB>::Ptr processed(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr filterx(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtery(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr filterz(new pcl::PointCloud<pcl::PointXYZRGB>);

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
  ros::init (argc, argv, "pcd2pointcloud2");
  ros::NodeHandle n;
  pcl::io::loadPCDFile<pcl::PointXYZRGB>("/home/jacky/catkin_ws/pcd/dsub1.pcd", *pcdsource);

  int i=0;
  // Remove NaN pointss
  std::vector<int> mapping;
  pcl::removeNaNFromPointCloud(*pcdsource, *processed, mapping);
  #ifdef VERBOSE
    std::cout << "Remove nan: " << processed->points.size() << "\n";
  #endif

  //pcl::io::loadPCDFile<pcl::PointXYZRGB>("/home/jacky/catkin_ws/rosbag/icp/1/1574148168244146.pcd", *pcdtarget);

  // Create ROS subscriber for fixed_frame topic
  //ros::Subscriber sub = n.subscribe("/pixel_to_xyz02/point_cloud",1,fixed_frame_cb);

  // Create ROS subscriber for camera_frame topic
  //ros::Subscriber sub2 = n.subscribe("/pixel_to_xyz04/point_cloud",1,camera_frame_cb);

  // Create ROS publisher for transformed pointcloud
  _pub = n.advertise<sensor_msgs::PointCloud2>("/pcd2pointcloud2",1);

  pcl::PassThrough<pcl::PointXYZRGB> passx;
  passx.setInputCloud (processed);
  passx.setFilterFieldName ("x");
  passx.setFilterLimits (-0.1, 0.1);
  //pass.setFilterLimitsNegative (true);
  passx.filter (*filterx);

  pcl::PassThrough<pcl::PointXYZRGB> passy;
  passy.setInputCloud (filterx);
  passy.setFilterFieldName ("y");
  passy.setFilterLimits (-0.02, 0.1);
  passy.filter (*filtery);

  pcl::PassThrough<pcl::PointXYZRGB> passz;
  passz.setInputCloud (filtery);
  passz.setFilterFieldName ("z");
  passz.setFilterLimits (0.0, 0.50);
  passz.filter (*filterz);

  printf("%d \n",filterz->width);
  printf("%d \n",filterz->height);
  printf("%d \n",filterz->points.size());

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;

  for(i=0;i < filterz->points.size();i++){
    if(filterz->points[i].b < filterz->points[i].g || filterz->points[i].b < filterz->points[i].r || filterz->points[i].r > 80){
        inliers->indices.push_back(i);
    }
  }

  extract.setInputCloud(filterz);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*filterz);
/*
  for(i=0;i < filterz->points.size();i++){
    printf("r : %d ",filterz->points[i].r);
    printf("g : %d ",filterz->points[i].g);
    printf("b : %d \n",filterz->points[i].b);
  }
  */
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  	Eigen::Vector4f pcaCentroid;
	pcl::compute3DCentroid(*filterz, pcaCentroid);
	Eigen::Matrix3f covariance;
	pcl::computeCovarianceMatrixNormalized(*filterz, pcaCentroid, covariance);
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
	Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
	Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();
	eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1)); //校正主方向间垂直
	eigenVectorsPCA.col(0) = eigenVectorsPCA.col(1).cross(eigenVectorsPCA.col(2));
	eigenVectorsPCA.col(1) = eigenVectorsPCA.col(2).cross(eigenVectorsPCA.col(0));
 
	std::cout << "特征值va(3x1):\n" << eigenValuesPCA << std::endl;
	std::cout << "特征向量ve(3x3):\n" << eigenVectorsPCA << std::endl;
	std::cout << "质心点(4x1):\n" << pcaCentroid << std::endl;
	/*
	// 另一种计算点云协方差矩阵特征值和特征向量的方式:通过pcl中的pca接口，如下，这种情况得到的特征向量相似特征向量
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCAprojection (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCA<pcl::PointXYZ> pca;
	pca.setInputCloud(cloudSegmented);
	pca.project(*cloudSegmented, *cloudPCAprojection);
	std::cerr << std::endl << "EigenVectors: " << pca.getEigenVectors() << std::endl;//计算特征向量
	std::cerr << std::endl << "EigenValues: " << pca.getEigenValues() << std::endl;//计算特征值
	*/
	Eigen::Matrix4f tm = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f tm_inv = Eigen::Matrix4f::Identity();
	tm.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();   //R.
	tm.block<3, 1>(0, 3) = -1.0f * (eigenVectorsPCA.transpose()) *(pcaCentroid.head<3>());//  -R*t
	tm_inv = tm.inverse();
 
	std::cout << "变换矩阵tm(4x4):\n" << tm << std::endl;
	std::cout << "逆变矩阵tm'(4x4):\n" << tm_inv << std::endl;
 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::transformPointCloud(*filterz, *transformedCloud, tm);
 
	pcl::PointXYZRGB min_p1, max_p1;
	Eigen::Vector3f c1, c;
	pcl::getMinMax3D(*transformedCloud, min_p1, max_p1);
	c1 = 0.5f*(min_p1.getVector3fMap() + max_p1.getVector3fMap());
 
	std::cout << "型心c1(3x1):\n" << c1 << std::endl;
 
	Eigen::Affine3f tm_inv_aff(tm_inv);
	pcl::transformPoint(c1, c, tm_inv_aff);
 
	Eigen::Vector3f whd, whd1;
	whd1 = max_p1.getVector3fMap() - min_p1.getVector3fMap();
	whd = whd1;
	float sc1 = (whd1(0) + whd1(1) + whd1(2)) / 3;  //点云平均尺度，用于设置主方向箭头大小
 
	std::cout << "width1=" << whd1(0) << endl;
	std::cout << "heght1=" << whd1(1) << endl;
	std::cout << "depth1=" << whd1(2) << endl;
	std::cout << "scale1=" << sc1 << endl;
 
	const Eigen::Quaternionf bboxQ1(Eigen::Quaternionf::Identity());
	const Eigen::Vector3f    bboxT1(c1);
 
	const Eigen::Quaternionf bboxQ(tm_inv.block<3, 3>(0, 0));
	const Eigen::Vector3f    bboxT(c);
 
 
	//变换到原点的点云主方向
	pcl::PointXYZRGB op;
	op.x = 0.0;
	op.y = 0.0;
	op.z = 0.0;
	Eigen::Vector3f px, py, pz;
	Eigen::Affine3f tm_aff(tm);
	pcl::transformVector(eigenVectorsPCA.col(0), px, tm_aff);
	pcl::transformVector(eigenVectorsPCA.col(1), py, tm_aff);
	pcl::transformVector(eigenVectorsPCA.col(2), pz, tm_aff);
	pcl::PointXYZRGB pcaX;
	pcaX.x = sc1 * px(0);
	pcaX.y = sc1 * px(1);
	pcaX.z = sc1 * px(2);
	pcl::PointXYZRGB pcaY;
	pcaY.x = sc1 * py(0);
	pcaY.y = sc1 * py(1);
	pcaY.z = sc1 * py(2);
	pcl::PointXYZRGB pcaZ;
	pcaZ.x = sc1 * pz(0);
	pcaZ.y = sc1 * pz(1);
	pcaZ.z = sc1 * pz(2);
 
 
	//初始点云的主方向
	pcl::PointXYZRGB cp;
	cp.x = pcaCentroid(0);
	cp.y = pcaCentroid(1);
	cp.z = pcaCentroid(2);
	pcl::PointXYZRGB pcX;
	pcX.x = sc1 * eigenVectorsPCA(0, 0) + cp.x;
	pcX.y = sc1 * eigenVectorsPCA(1, 0) + cp.y;
	pcX.z = sc1 * eigenVectorsPCA(2, 0) + cp.z;
	pcl::PointXYZRGB pcY;
	pcY.x = sc1 * eigenVectorsPCA(0, 1) + cp.x;
	pcY.y = sc1 * eigenVectorsPCA(1, 1) + cp.y;
	pcY.z = sc1 * eigenVectorsPCA(2, 1) + cp.z;
	pcl::PointXYZRGB pcZ;
	pcZ.x = sc1 * eigenVectorsPCA(0, 2) + cp.x;
	pcZ.y = sc1 * eigenVectorsPCA(1, 2) + cp.y;
	pcZ.z = sc1 * eigenVectorsPCA(2, 2) + cp.z;
 
 
	//visualization
	pcl::visualization::PCLVisualizer viewer;
 
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> tc_handler(transformedCloud, 0, 255, 0); //转换到原点的点云相关
	viewer.addPointCloud(transformedCloud, tc_handler, "transformCloud");
	viewer.addCube(bboxT1, bboxQ1, whd1(0), whd1(1), whd1(2), "bbox1");
	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "bbox1");
	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "bbox1");
 
	viewer.addArrow(pcaX, op, 1.0, 0.0, 0.0, false, "arrow_X");
	viewer.addArrow(pcaY, op, 0.0, 1.0, 0.0, false, "arrow_Y");
	viewer.addArrow(pcaZ, op, 0.0, 0.0, 1.0, false, "arrow_Z");
 
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> color_handler(filterz, 255, 0, 0);  //输入的初始点云相关
	viewer.addPointCloud(filterz, color_handler, "cloud");
	viewer.addCube(bboxT, bboxQ, whd(0), whd(1), whd(2), "bbox");
	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "bbox");
	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "bbox");
 
	viewer.addArrow(pcX, cp, 1.0, 0.0, 0.0, false, "arrow_x");
	viewer.addArrow(pcY, cp, 0.0, 1.0, 0.0, false, "arrow_y");
	viewer.addArrow(pcZ, cp, 0.0, 0.0, 1.0, false, "arrow_z");
 
	viewer.addCoordinateSystem(0.5f*sc1);
	viewer.setBackgroundColor(1.0, 1.0, 1.0);
	/*
	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
	}
*/
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // Convert the pcl/PointCloud to sensor_msgs/PointCloud2
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*filterz, output);
  output.header.frame_id = "base_link";
  while(ros::ok()){
	// Publish the results
  	//ROS_INFO("%s ", output.header.frame_id.c_str());
  	_pub.publish(output);

  	// Spin
  	ros::spinOnce();
  	ros::Duration(1).sleep();
  }
  return 0;
}
