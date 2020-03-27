#include "dataPrepare.h"
#include <fstream>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace global_localization
{
dataPrepare::dataPrepare(bool pubRGBCloud) : nh_(ros::this_node::getName()), it_(nh_), vo_(), pubRGBCloud_(pubRGBCloud),
    cloudRGB_(new pcl::PointCloud<pcl::PointXYZRGB>), cloud_(new pcl::PointCloud<pcl::PointXYZ>)
{
}
dataPrepare::dataPrepare() : nh_(ros::this_node::getName()), it_(nh_), vo_(), pubRGBCloud_(false),
    cloudRGB_(new pcl::PointCloud<pcl::PointXYZRGB>), cloud_(new pcl::PointCloud<pcl::PointXYZ>)
{
}
dataPrepare::~dataPrepare()
{
  ROS_DEBUG("[%s] Node::~Node()", ros::this_node::getName().data());
}
void dataPrepare::colorCallback(const sensor_msgs::ImageConstPtr& msg)
{
    //ROS_INFO("color  callback open");
    //cout<<msg->header.stamp<<endl;
    double h_time = msg->header.stamp.toSec();
    try
    {
        color_ = cv_bridge::toCvShare(msg, "bgr8")->image;
        vo_.setColor(color_);
        // color msg comes later 
        if(!vo_.isInitialized()){
            vo_.init();
        }
        else{
            cv::Mat pose = vo_.estimatePose(h_time);
            
            ros::Time current_time = ros::Time::now(); //
            //ros::Time current_time = msg->header.stamp;  // if use_sim_time true
            
            //cout<<pose<<endl;
            Eigen::Matrix3d rot;
            rot << pose.at<double>(0,0), pose.at<double>(0,1), pose.at<double>(0,2), 
                   pose.at<double>(1,0), pose.at<double>(1,1), pose.at<double>(1,2), 
                   pose.at<double>(2,0), pose.at<double>(2,1), pose.at<double>(2,2);
            
            Eigen::Vector3d eulerAngle = rot.eulerAngles(2,1,0);
            //cout<<rot<<endl;
            
            //since all odometry is 6DOF we'll need a quaternion created from yaw
            geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(eulerAngle(0), eulerAngle(1), eulerAngle(2));

            //first, we'll publish the transform over tf
            geometry_msgs::TransformStamped odomTrans;
            odomTrans.header.stamp = current_time;
            odomTrans.header.frame_id = "odom";
            odomTrans.child_frame_id = "base_link";

            odomTrans.transform.translation.x = pose.at<double>(0,3);
            odomTrans.transform.translation.y = pose.at<double>(1,3);
            odomTrans.transform.translation.z = pose.at<double>(2,3);
            odomTrans.transform.rotation = odom_quat;

            odomBroadcaster_.sendTransform(odomTrans);

            //next, we'll publish the odometry message over ROS
            nav_msgs::Odometry odom;
            odom.header.stamp = current_time;
            odom.header.frame_id = "odom";

            //set the position
            odom.pose.pose.position.x = pose.at<double>(0,3);
            odom.pose.pose.position.y = pose.at<double>(1,3);
            odom.pose.pose.position.z = pose.at<double>(2,3);
            odom.pose.pose.orientation = odom_quat;

            //not set the velocity for simplicity
            odom.child_frame_id = "base_link";
            odom.twist.twist.linear.x = 0;
            odom.twist.twist.linear.y = 0;
            odom.twist.twist.angular.z = 0;

            //publish the message
            odomPub_.publish(odom);
        }
            
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
    //ROS_INFO("color    callback end");
}
void dataPrepare::depthCallback(const sensor_msgs::ImageConstPtr& msg)
{
    //ROS_INFO("depth  callback open");
    //cout<<msg->header.stamp<<endl;
    try
    {
        depth_ = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_32FC1)->image;
        vo_.setDepth(depth_);
        
        for(int m = 0; m < depth_.rows; m++){
            for(int n = 0; n < depth_.cols; n++){
                float d = depth_.ptr<float>(m)[n];

                if(d <= 0.00001 || isnan(d))
                    continue;

                pcl::PointXYZ point;
                point.z = double (d);
                point.x = (n - camera_cx_) * point.z / camera_fx_;
                point.y = (m - camera_cy_) * point.z / camera_fy_;
                cloud_->points.push_back(point);

                if(pubRGBCloud_){
                    //color image comes after depth image so this will cause segment fault
                    if(color_.empty()){
                        return;
                    }
                
                    pcl::PointXYZRGB p;
                    p.z = point.z;
                    p.x = point.x;
                    p.y = point.y;
                    p.b = color_.ptr<uchar>(m)[n*3];
                    p.g = color_.ptr<uchar>(m)[n*3+1];
                    p.r = color_.ptr<uchar>(m)[n*3+2];
                    cloudRGB_->points.push_back(p);
                }
            }
        }
    
        cloud_->height = 1;
        cloud_->width = cloud_->points.size();
        //ROS_INFO("point cloud size = %d",cloud_->width);
        cloud_->is_dense = false;
        pcl::toROSMsg(*cloud_, pointcloudMsg_);
        pointcloudMsg_.header.frame_id = "point_cloud";
        pointcloudMsg_.header.stamp = ros::Time::now();

        pointcloudPub_.publish(pointcloudMsg_);
        cloud_->points.clear();
        
        if(pubRGBCloud_){
            cloudRGB_->height = 1;
            cloudRGB_->width = cloudRGB_->points.size();
            //ROS_INFO("point cloud size = %d",cloudRGB_->width);
            cloudRGB_->is_dense = false;
            pcl::toROSMsg(*cloudRGB_, pointcloudRGBMsg_);
            pointcloudRGBMsg_.header.frame_id = "kinect";
            pointcloudRGBMsg_.header.stamp = ros::Time::now();

            pointcloudRGBPub_.publish(pointcloudRGBMsg_);
            cloudRGB_->points.clear();
        }
        
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'mono16'.", msg->encoding.c_str());
    }
    //ROS_INFO("callback end");
}
void dataPrepare::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& cam_info)
{
    cv::Mat K = (cv::Mat_<double>(3, 3) << 
        cam_info->K[0], cam_info->K[1], cam_info->K[2],
        cam_info->K[3], cam_info->K[4], cam_info->K[5],
        cam_info->K[6], cam_info->K[7], cam_info->K[8]);
    
    camera_cx_ = cam_info->K[2];
    camera_cy_ = cam_info->K[5];
    camera_fx_ = cam_info->K[0];
    camera_fy_ = cam_info->K[4];
    if(vo_.noInternalPara())
        vo_.setInternalPara(K);
}
void readAssocTextfile(std::string filename,
                       std::vector<std::string>& inputRGBPaths,
                       std::vector<std::string>& inputDepthPaths,
                       std::vector<std::string>& inputTimeStamps,
                       std::vector<float>& pose) {
    std::string line;
    std::ifstream in_stream(filename.c_str());
    
    bool readPose(false);
    while (!in_stream.eof()) {
        std::getline(in_stream, line);
        std::stringstream ss(line);
        std::string buf;
        int count = 0;
        while (ss >> buf) {
            count++;
            if (count == 2) {
                inputRGBPaths.push_back(buf);
            } else if (count == 4) {
                inputDepthPaths.push_back(buf);
            } else if (count == 1) {
                inputTimeStamps.push_back(buf);
            } 
            if(count == 4){
                //get the first frame pose in the world
                if(readPose)
                    break;
                else{
                    ss >> buf;
                    for(int i=0; i<7; i++){
                        ss >> buf;
                        pose.push_back(atof(buf.c_str()));
                        std::cout<<pose[i]<<" ";
                    }
                    std::cout<<std::endl;
                    readPose = true;
                }
            }
        }
    }
    in_stream.close();
}
bool dataPrepare::configure()
{
    std::vector<std::string> inputRGBPaths, inputDepthPaths, inputTimeStamps;
    std::vector<float> pose;
    readAssocTextfile("/home/limz/data/rgbd-data/TUM/rgbd_dataset_freiburg1_xyz/allData.txt", inputRGBPaths, inputDepthPaths, inputTimeStamps, pose);
    vo_.setOrign(pose);
    
#ifdef ROSMODE
    colorSub_ = it_.subscribe("/camera/rgb/image_color", 100, &dataPrepare::colorCallback, this);
    depthSub_ = it_.subscribe("/camera/depth/image", 100, &dataPrepare::depthCallback, this);
    
    cameraInfoSub_ = nh_.subscribe("/camera/depth/camera_info", 1, &dataPrepare::cameraInfoCallback, this);
    
    odomPub_ = nh_.advertise<nav_msgs::Odometry>("odom", 50);
    pointcloudPub_ = nh_.advertise<sensor_msgs::PointCloud2>("/point_cloud", 1);
    pointcloudRGBPub_ = nh_.advertise<sensor_msgs::PointCloud2>("/point_cloudRGB", 1);
    
#else
    vo_.playSequence(inputRGBPaths, inputDepthPaths, inputTimeStamps);
#endif

    return true;
}

}  // namespace amcl3d
