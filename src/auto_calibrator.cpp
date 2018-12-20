#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/PointCloud2.h>

#include "std_msgs/Float32MultiArray.h"

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <stdio.h>
#include <time.h>
#include <iostream>

#include <Eigen/Dense>
#include <Eigen/Geometry>


#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/io/pcd_io.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/ModelCoefficients.h>

using namespace Eigen;
class AutoCalibrator
{
private:
  ros::NodeHandle _nh;
  ros::Subscriber _sub_pcd;
  ros::Publisher  _pub_pcd      = _nh.advertise<sensor_msgs::PointCloud2> ("/output", 5);
  ros::Publisher  _pub_vertex   = _nh.advertise<std_msgs::Float32MultiArray> ("/vertex", 10);

  tf::TransformListener tfListener;
  tf::TransformBroadcaster tfBroadcaster;

  sensor_msgs::PointCloud2 output_cloud;

  double downSamplingResolution   = 0.2; //[m] 
  double planeDetectionDistanceTh = 0.05; //[m] 

public:
  AutoCalibrator()
  {
    // subscribe ROS topics
    _sub_pcd   = _nh.subscribe("/camera/depth/points", 1, &AutoCalibrator::savePCD, this);
    ROS_INFO("Listening for incoming data on topic /hsrb/head_rgbd_sensor/depth_registered/rectified_points ...");
  }
  ~AutoCalibrator() {}
  
  // get points
  void savePCD(const sensor_msgs::PointCloud2ConstPtr &input_cloud)
  {
    // Container for original & filtered data
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudDS(cloud);
    pcl::PCLPointCloud2 cloud_filtered;

    // Convert to PCL data type
    pcl_conversions::toPCL(*input_cloud, *cloud);

    //Down sampling
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloudDS);
    sor.setLeafSize (downSamplingResolution, downSamplingResolution, downSamplingResolution);
    sor.filter (cloud_filtered);
    //pcl_conversions::fromPCL(cloud_filtered, output_cloud);
    
    //Plane detection
    pcl::PointCloud<pcl::PointXYZ> cloudPD;
    pcl::fromPCLPointCloud2(cloud_filtered, cloudPD);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients (true);

    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (planeDetectionDistanceTh);

    seg.setInputCloud (cloudPD.makeShared());
    seg.segment (*inliers, *coefficients);

    //Get projection area
    if(inliers->indices.size() == 0){
      std::cerr << "Could not estimate a planar model for the given data" << std::endl;
    }
    else
    {
      double planeCoefficients[4];
      for(int i=0;i<4;i++)
      {
        planeCoefficients[i] = coefficients->values[i];
      }
      this->getProjectionArea(planeCoefficients);
    }
  }
  void getProjectionArea(double *planeCoefficients)
  {
    Vector3f c;//平面とカメラの光軸との交点
    c<<0,0,-planeCoefficients[3]/planeCoefficients[2];

    Vector3f n;//平面の法線ベクトル
    n << planeCoefficients[0],planeCoefficients[1],planeCoefficients[2];

    Vector3f ey;//平面上の一つのベクトル1
    VectorXf ez(3);//平面上の一つのベクトル2

    ey << 0,-n(2)/n(1)*sqrt(1/(1+pow(n(2)/n(1),2))),sqrt(1/(1+pow(n(2)/n(1),2)));

    Matrix3f rot;//回転行列
    rot = AngleAxisf(M_PI/2,ey);//法線軸周りで90°回転する回転行列
    ez = rot*n;

    double  sideLength = 0.2; //一辺の長さ(0.20[m]))
    Vector3f V1;//四角形の各頂点
    Vector3f V2;
    Vector3f V3;
    Vector3f V4;
    V1 = c + sideLength*ey - sideLength/2*ez;
    V2 = c + sideLength*ey + sideLength/2*ez;
    V3 = c + sideLength/2*ez;
    V4 = c - sideLength/2*ez;

    /*
    tf::Transform transform;  
    transform.setOrigin( tf::Vector3(V1(0), V1(1), V1(2)) );
    tf::Quaternion quaternion;
    quaternion.setRPY(0, 0, 0);
    transform.setRotation(quaternion);
    transform.setOrigin( tf::Vector3(V1(0), V1(1), V1(2)) );
    tfBroadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_depth_optical_frame", "V1"));

    transform.setOrigin( tf::Vector3(V2(0), V2(1), V2(2)) );
    tfBroadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_depth_optical_frame", "V2"));

    transform.setOrigin( tf::Vector3(V3(0), V3(1), V3(2)) );
    tfBroadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_depth_optical_frame", "V3"));

    transform.setOrigin( tf::Vector3(V4(0), V4(1), V4(2)) );
    tfBroadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_depth_optical_frame", "V4"));
    */

    //プロジェクター座標系に座標変換
    //動作周波数向上のため、変換にはTFを使用しない
    tf::StampedTransform projector_transform;
    try
    {
      tfListener.lookupTransform("/camera_depth_optical_frame", "/projector_frame", ros::Time(0), projector_transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }
    Vector3f projector_pos;
    projector_pos << projector_transform.getOrigin().x(),
                     projector_transform.getOrigin().y(),
                     projector_transform.getOrigin().z();

    Vector3f P1;//四角形の各頂点
    Vector3f P2;
    Vector3f P3;
    Vector3f P4;
    P1 = V1-projector_pos;
    P2 = V2-projector_pos;
    P3 = V3-projector_pos;
    P4 = V4-projector_pos;

    tf::Transform transform;  
    transform.setOrigin( tf::Vector3(V1(0), V1(1), V1(2)) );
    tf::Quaternion quaternion;
    quaternion.setRPY(0, 0, 0);
    transform.setRotation(quaternion);

    transform.setOrigin( tf::Vector3(P1(0), P1(1), P1(2)) );
    tfBroadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "projector_frame", "P1"));

    transform.setOrigin( tf::Vector3(P2(0), P2(1), P2(2)) );
    tfBroadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "projector_frame", "P2"));

    transform.setOrigin( tf::Vector3(P3(0), P3(1), P3(2)) );
    tfBroadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "projector_frame", "P3"));

    transform.setOrigin( tf::Vector3(P4(0), P4(1), P4(2)) );
    tfBroadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "projector_frame", "P4"));

    double projection_distance=0.9;
    std_msgs::Float32MultiArray vertexs;
    vertexs.data.push_back(P1(0)/P1(2)*projection_distance);
    vertexs.data.push_back(P1(1)/P1(2)*projection_distance);
    vertexs.data.push_back(P2(0)/P2(2)*projection_distance);
    vertexs.data.push_back(P2(1)/P2(2)*projection_distance);
    vertexs.data.push_back(P3(0)/P3(2)*projection_distance);
    vertexs.data.push_back(P3(1)/P3(2)*projection_distance);
    vertexs.data.push_back(P4(0)/P4(2)*projection_distance);
    vertexs.data.push_back(P4(1)/P4(2)*projection_distance);
    _pub_vertex.publish(vertexs);    
  }
};
int main(int argc, char **argv)
{
  ros::init(argc, argv, "auto_calibrator");
 
  AutoCalibrator autoCalibrator;
  
  ros::spin();
}