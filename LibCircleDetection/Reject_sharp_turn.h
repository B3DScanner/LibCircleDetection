#pragma once
//Reject sharp turn angles

#include<iostream>
#include<vector>
#include<opencv2/opencv.hpp>
#include<math.h>

//using namespace std;
//using namespace cv;

namespace Zikai {
	struct sharpTurn
	{
		std::vector<std::vector<cv::Point>> new_edgeList;
		std::vector<std::vector<cv::Point>> new_segList;
	};


	sharpTurn rejectSharpTurn(
		const std::vector<std::vector<cv::Point>>& edgeList,
		const std::vector<std::vector<cv::Point>>& segList,
		float angle
	);
}





