#include "vo.h"

#include <fstream>
#include <string>
#include <cstdlib>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>


using namespace std;

namespace global_localization
{

Eigen::Matrix<double,3,1> toVector3d(const cv::Mat &cvVector)
{
    Eigen::Matrix<double,3,1> v;
    v << cvVector.at<double>(0), cvVector.at<double>(1), cvVector.at<double>(2);

    return v;
}

Eigen::Matrix<double,3,3> toMatrix3d(const cv::Mat &cvMat3)
{
    Eigen::Matrix<double,3,3> M;

    M << cvMat3.at<double>(0,0), cvMat3.at<double>(0,1), cvMat3.at<double>(0,2),
    cvMat3.at<double>(1,0), cvMat3.at<double>(1,1), cvMat3.at<double>(1,2),
    cvMat3.at<double>(2,0), cvMat3.at<double>(2,1), cvMat3.at<double>(2,2);

    return M;
}
void VO::setOrign(const std::vector<float>& pose){

    Eigen::Quaternionf q(pose[6], pose[3], pose[4], pose[5]); 
//    cout << q.x() << endl << endl;
//    cout << q.y() << endl << endl;
//    cout << q.z() << endl << endl;
//    cout << q.w() << endl << endl;
    
    Eigen::Matrix3f Rx = q.toRotationMatrix();
//    cout << Rx << endl << endl;

    Two_ = (cv::Mat_<double>(4, 4) << 
                Rx(0,0), Rx(0,1), Rx(0,2), pose[0], 
                Rx(1,0), Rx(1,1), Rx(1,2), pose[1], 
                Rx(2,0), Rx(2,1), Rx(2,2), pose[2], 
                0, 0, 0, 1);
//    cout<<Two_<<endl;
}
VO::VO() : rosMode_(true)
{
    camera_cx_ = 319.5;
    camera_cy_ = 239.5;
    camera_fx_ = 525;
    camera_fy_ = 525;
    
    K_ = (cv::Mat_<double>(3, 3) << 
                525, 0, 319.5,
                0, 525, 239.5,
                0, 0, 1);
        
    traj_ = cv::Mat::zeros(600, 600, CV_8UC3);
}
VO::~VO(){
    f_.close();
    std::cout<<"close."<<std::endl;
}
void VO::setColor(const cv::Mat& color){
    color.copyTo(color_);
}

void VO::setDepth(const cv::Mat& depth){
    depth.copyTo(depth_);  
}
void VO::init(){
    cout<<"initial VO..."<<endl;
    
	extractKeyPoints(color_, depth_, mapPoints_, featurePoints_);

    std::string fname = "voPosesBag.txt";
    f_.open(fname.c_str(), std::fstream::out);
    
	color_.copyTo(refImg_);
	
    initialize_ = true;
}

cv::Mat VO::estimatePose(double hTime){
    index_++;
    
	cv::imshow("image", color_);
	cv::imshow("depth", depth_);
	
	std::vector<double> pose;
	
	std::vector<cv::Point3f> mapPointsRef;
	std::vector<cv::Point2f> featurePointsRef;

	tracking(refImg_, color_, featurePoints_, mapPoints_, mapPointsRef, featurePointsRef);

	if(mapPointsRef.size() == 0) 
	    return cv::Mat();

	cv::Mat dist_coeffs = cv::Mat::zeros(4, 1, CV_64F);
	cv::Mat rvec, tvec;
	
	vector<int> inliers;
	//3d frame to 2d frame
	cv::solvePnPRansac(mapPointsRef, featurePointsRef, K_, dist_coeffs, rvec, tvec, false, 100, 8.0, 0.99, inliers);

	if(inliers.size() < 5)
	    return cv::Mat();

	float inliers_ratio = inliers.size()/float(mapPointsRef.size());
    
	cv::Mat R_matrix;
	cv::Rodrigues(rvec, R_matrix); 
	R_matrix = R_matrix.t();
	cv::Mat t_vec = -R_matrix*tvec;
    
	cv::Mat inv_transform = cv::Mat::eye(4,4,CV_64F);
	R_matrix.copyTo(inv_transform.rowRange(0,3).colRange(0,3));
	t_vec.copyTo(inv_transform.rowRange(0,3).col(3));

    createNewPoints(color_, depth_, inv_transform, featurePoints_, mapPoints_);
    
    //save pose.txt
    std::stringstream strs;
    strs << std::setprecision(6) << std::fixed << hTime << " ";
    
    cv::Mat Twc = Two_ * inv_transform;
    
    Eigen::Vector3d trans = toVector3d(Twc.rowRange(0,3).col(3));
    Eigen::Matrix3d rot = toMatrix3d(Twc.rowRange(0,3).colRange(0,3));
    
    f_ << strs.str() << trans(0) << " " << trans(1) << " " << trans(2) << " ";
    //cout<<index_<<" "<<trans(0) << " " << trans(1) << " " << trans(2) <<endl;
    
    Eigen::Quaterniond currentCameraRotation(rot);
    f_ << currentCameraRotation.x() << " " << currentCameraRotation.y() << " " << currentCameraRotation.z() << " " << currentCameraRotation.w() << "\n";

    for(int i=0; i<3; i++)
        pose.push_back(trans[i]);
    pose.push_back(currentCameraRotation.x());
    pose.push_back(currentCameraRotation.y());
    pose.push_back(currentCameraRotation.z());
    pose.push_back(currentCameraRotation.w());
        
    //note must depth copy
	color_.copyTo(refImg_);

	// plot the information
	string text  = "Red color: estimated trajectory";

	t_vec.convertTo(t_vec, CV_32F);

	cv::Point2f center = cv::Point2f(int(60*t_vec.at<float>(0)) + 300, int(60*t_vec.at<float>(2)) + 100);

	cv::circle(traj_, center ,1, cv::Scalar(0,0,255), 2);
	cv::rectangle(traj_, cv::Point2f(10, 30), cv::Point2f(550, 50),  cv::Scalar(0,0,0), cv::FILLED);
	putText(traj_, text, cv::Point2f(10,50), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0,255), 1, 5);
	cv::imshow( "Trajectory", traj_);
	cv::waitKey(1);
	
	return inv_transform;
}
void VO::extractKeyPoints(const cv::Mat& color, const cv::Mat& depth, std::vector<cv::Point3f>&mapPoints, std::vector<cv::Point2f>& featurePoints) {
    // Feature Detection and Extraction
	cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create(350);

	vector<cv::KeyPoint> keypoints;
	detector->detect(color, keypoints);

    for(auto kp:keypoints){
        cv::Point2f p = kp.pt;
        
        float d;
        if(rosMode_){
            d = depth_.ptr<float>((int)p.y)[(int)p.x];
        }
        else{
            ushort dtemp = depth.ptr<ushort>((int)p.y)[(int)p.x];
            d = double(dtemp)/5000;
        }
        
        //nan is zero in raw image
        if(d <= 0.00001 || isnan(d))
            continue;
        
        cv::Point3f p3;
        float z = d;
        float x = (p.x - camera_cx_) * z / camera_fx_;
        float y = (p.y - camera_cy_) * z / camera_fy_;
        //cout<<"3d: "<<x<<" "<<y<<" "<<z<<endl;
        
        featurePoints.push_back(p);
        mapPoints.push_back(cv::Point3f(x, y, z));
    }
}
void VO::tracking(const cv::Mat& ref_img, const cv::Mat& curImg, const std::vector<cv::Point2f>& featurePoints, const std::vector<cv::Point3f>& mapPoints, std::vector<cv::Point3f>& mapPointsRef, std::vector<cv::Point2f>& featurePointsRef) {

	vector<cv::Point2f> nextPts;
	vector<uchar> status;
	vector<float> err;

	cv::calcOpticalFlowPyrLK(ref_img, curImg, featurePoints, nextPts, status, err);

	for (int  j = 0; j < status.size(); j++) {
		if (status[j] == 1) {
			featurePointsRef.push_back(nextPts[j]);
			mapPointsRef.push_back(mapPoints[j]);
		}
	}
}
void VO::createNewPoints(const cv::Mat& color, const cv::Mat& depth, const cv::Mat& inv_transform, std::vector<cv::Point2f>& featurePoints, std::vector<cv::Point3f>& mapPoints){

	if(featurePoints.size() != 0){
		featurePoints.clear();
		mapPoints.clear();
	}
    
	vector<cv::Point3f>  mapPointsNew;
	vector<cv::Point2f>  featurePointsNew;

	extractKeyPoints(color, depth, mapPointsNew, featurePointsNew);

	// // cout << inv_transform << endl;

   for (int k = 0; k < mapPointsNew.size(); k++) {
// 
	   	const cv::Point3f& pt = mapPointsNew[k];

	   	cv::Point3f p;

	   	p.x = inv_transform.at<double>(0, 0)*pt.x + inv_transform.at<double>(0, 1)*pt.y + inv_transform.at<double>(0, 2)*pt.z + inv_transform.at<double>(0, 3);
	   	p.y = inv_transform.at<double>(1, 0)*pt.x + inv_transform.at<double>(1, 1)*pt.y + inv_transform.at<double>(1, 2)*pt.z + inv_transform.at<double>(1, 3);
	   	p.z = inv_transform.at<double>(2, 0)*pt.x + inv_transform.at<double>(2, 1)*pt.y + inv_transform.at<double>(2, 2)*pt.z + inv_transform.at<double>(2, 3);

	   	// cout << p << endl;
	   	if (p.z > 0) {
			mapPoints.push_back(p);
			featurePoints.push_back(featurePointsNew[k]);
	   	}

   }
}
void VO::playSequence(const std::vector<std::string>& inputRGBPaths,
                      const std::vector<std::string>& inputDepthPaths,
                      const std::vector<std::string>& inputTimeStamps) {

    cv::Mat color = cv::imread(inputRGBPaths[0]);
    cv::Mat depth = cv::imread(inputDepthPaths[0], -1);
    
	cv::Mat& ref_img  = color;

	vector<cv::Point3f> mapPoints;
	vector<cv::Point2f> featurePoints;

	extractKeyPoints(color, depth, mapPoints, featurePoints);

	cv::Mat curImg;
	cv::Mat traj = cv::Mat::zeros(600, 600, CV_8UC3);

    std::string fname = "voPoses.txt";
    std::ofstream f;
    f.open(fname.c_str(), std::fstream::out);
	for(int i=1; i<inputRGBPaths.size(); i++){

		curImg = cv::imread(inputRGBPaths[i]);
		cv::imshow( "image", curImg);
		
		std::vector<cv::Point3f> mapPointsRef;
		std::vector<cv::Point2f> featurePointsRef;

		tracking(ref_img, curImg, featurePoints, mapPoints, mapPointsRef, featurePointsRef);

		if(mapPointsRef.size() == 0) 
		    continue;

		cv::Mat dist_coeffs = cv::Mat::zeros(4, 1, CV_64F);
		cv::Mat rvec, tvec;
		
		vector<int> inliers;
		//3d frame to 2d frame
 		cv::solvePnPRansac(mapPointsRef, featurePointsRef, K_, dist_coeffs,rvec, tvec, false, 100, 8.0, 0.99, inliers);

		if(inliers.size() < 5) 
		    continue;

		float inliers_ratio = inliers.size()/float(mapPointsRef.size());

		//cout << "inliers ratio: " << inliers_ratio << endl;

		cv::Mat R_matrix;
		cv::Rodrigues(rvec,R_matrix); 
		R_matrix = R_matrix.t();
		cv::Mat t_vec = -R_matrix*tvec;

		cv::Mat inv_transform = cv::Mat::eye(4,4,CV_64F);
		R_matrix.copyTo(inv_transform.rowRange(0,3).colRange(0,3));
		t_vec.copyTo(inv_transform.rowRange(0,3).col(3));

        cv::Mat depth = cv::imread(inputDepthPaths[i], -1);
	    createNewPoints(curImg, depth, inv_transform, featurePoints, mapPoints);

        //save pose.txt
        std::stringstream strs;
        strs << std::setprecision(6) << std::fixed << inputTimeStamps[i] << " ";
        
        cv::Mat Twc = Two_ * inv_transform;
        
        Eigen::Vector3d trans = toVector3d(Twc.rowRange(0,3).col(3));
        Eigen::Matrix3d rot = toMatrix3d(Twc.rowRange(0,3).colRange(0,3));
        
        f << strs.str() << trans(0) << " " << trans(1) << " " << trans(2) << " ";
        
        Eigen::Quaterniond currentCameraRotation(rot);
        f << currentCameraRotation.x() << " " << currentCameraRotation.y() << " " << currentCameraRotation.z() << " " << currentCameraRotation.w() << "\n";
    
    
		ref_img = curImg;

		// plot the information
		string text  = "Red color: estimated trajectory";

		t_vec.convertTo(t_vec, CV_32F);
		//cout << t_vec.t() << endl;
        //cout<<t_vec.at<float>(0)<<" "<<t_vec.at<float>(1)<<" "<<t_vec.at<float>(2)<<endl;

		cv::Point2f center = cv::Point2f(int(60*t_vec.at<float>(0)) + 300, int(60*t_vec.at<float>(2)) + 100);

		cv::circle(traj, center ,1, cv::Scalar(0,0,255), 2);
		cv::rectangle(traj, cv::Point2f(10, 30), cv::Point2f(550, 50),  cv::Scalar(0,0,0), cv::FILLED);
		putText(traj, text, cv::Point2f(10,50), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0,255), 1, 5);
		cv::imshow( "Trajectory", traj);
		cv::waitKey(1);

	}
	f.close();

}

}
