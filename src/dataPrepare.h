#pragma once

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include "vo.h"

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

namespace global_localization
{
#define ROSMODE

class dataPrepare
{
public:
    dataPrepare();
    dataPrepare(bool pubRGBCloud);
    virtual ~dataPrepare();

    bool configure();

private:

    void colorCallback(const sensor_msgs::ImageConstPtr& msg);
    void depthCallback(const sensor_msgs::ImageConstPtr& msg);
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& cam_info);
    
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;

    VO vo_;
    
    cv::Mat color_, depth_;
    image_transport::Subscriber colorSub_, depthSub_;
    ros::Subscriber cameraInfoSub_;
    
    ros::Publisher odomPub_;
    ros::Publisher pointcloudPub_, pointcloudRGBPub_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRGB_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
    sensor_msgs::PointCloud2 pointcloudMsg_, pointcloudRGBMsg_;
  
    tf::TransformBroadcaster odomBroadcaster_;
    double camera_cx_;
    double camera_cy_;
    double camera_fx_;
    double camera_fy_;
    
    bool pubRGBCloud_;
};

}  // namespace global_localization
