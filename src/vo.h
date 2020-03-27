
#ifndef VO_H
#define VO_H


#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/features2d.hpp"

namespace global_localization
{
class VO {

public:

	VO();
	virtual ~VO();

    void init();
    cv::Mat estimatePose(double hTime);
    
    void tracking(const cv::Mat& ref_img, const cv::Mat& curImg, const std::vector<cv::Point2f>& featurePoints, const std::vector<cv::Point3f>& mapPoints, std::vector<cv::Point3f>& mapPointsRef, std::vector<cv::Point2f>& featurePointsRef);
    
    void createNewPoints(const cv::Mat& color, const cv::Mat& depth, const cv::Mat& inv_transform, std::vector<cv::Point2f>& featurePoints, std::vector<cv::Point3f>& mapPoints);

    void extractKeyPoints(const cv::Mat& color, const cv::Mat& depth, std::vector<cv::Point3f>& mapPoints, std::vector<cv::Point2f>& featurePoints);
    
	//this is to play on the image sequence
	void playSequence(const std::vector<std::string>& inputRGBPaths,
                      const std::vector<std::string>& inputDepthPaths,
                      const std::vector<std::string>& inputTimeStamps);

	cv::Mat getK() const {return K_;}

    void setTimeStamp(const std::vector<std::string>& inputTimeStamps){inputTimeStamps_ = inputTimeStamps;}
    void setOrign(const std::vector<float>& pose);
    void setColor(const cv::Mat& color);
    void setDepth(const cv::Mat& depth);
    bool noInternalPara(){ return K_.empty(); }
    void setInternalPara(const cv::Mat& K){
        K_ = K;
        camera_cx_ = K.ptr<double>(0)[2];
        camera_cy_ = K.ptr<double>(1)[2];
        camera_fx_ = K.ptr<double>(0)[0];
        camera_fy_ = K.ptr<double>(1)[1];
        
    }
    bool isInitialized(){ return initialize_;}
private:
	cv::Mat K_;
    double camera_cx_;
    double camera_cy_;
    double camera_fx_;
    double camera_fy_;

	bool initialize_{false};
	bool rosMode_;
	
	std::vector<cv::Point3f> mapPoints_;
	std::vector<cv::Point2f> featurePoints_;
	
	cv::Mat refImg_, depth_, color_;
	cv::Mat Two_;
	cv::Mat traj_;
	
	std::vector<std::string> inputTimeStamps_;
	std::ofstream f_;
	int index_{0};
};
}

#endif
