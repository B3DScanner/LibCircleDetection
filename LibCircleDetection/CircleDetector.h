#pragma once
#include <filesystem>
#include "ZikaiCircle.h"
#include <opencv2/opencv.hpp>

namespace Zikai
{

	// For better performance, you can mainly tune the parameters:  'T_inlier' and 'sharp_angle' 
	struct CircleDetectionThreshold {
		int T_l = 20;
		float T_ratio = 0.001;
		int T_o = 5;// 5 10 15 20 25
		int T_r = 5;// 5 10 15 20 25
		float T_inlier = 0.35;//0.3 0.35 0.4 0.45 0.5 (the larger the more strict)
		float T_angle = 2.0;// 
		float T_inlier_closed = 0.5;//0.5,0.6 0.7 0.8,0.9
		float sharp_angle = 60;//35 40 45 50 55 60 

	};

	class CircleDetector
	{
	public:


		inline void setEnableInfoPrinting(bool flag)
		{
			enableInforPrinting_ = flag;
		}
		inline void setEnableGaussianBlur(bool flag)
		{
			enableGaussianBlur_ = flag;
		}
	
		inline void setStepDebugDirectory(const std::filesystem::path& dirPath)
		{
			stepDebugDirectory_ = dirPath;
		}
	
		inline void setThreshold(const CircleDetectionThreshold& threshold)
		{
			threshold_ = threshold;
		}

		std::vector<Circle> detectCircles(const cv::Mat& inputImage);

		static cv::Mat drawCircles(
			const cv::Mat& inputImage,
			const std::vector<Circle>& detectedCircles
		);

	protected:

		void saveStepDebugImages() const;

		bool enableInforPrinting_ = false;
		bool enableGaussianBlur_ = true;
		

		CircleDetectionThreshold threshold_;

		std::filesystem::path stepDebugDirectory_;

		std::vector<std::pair<cv::Mat, std::string>> stepDebugImages_;
	};
}


