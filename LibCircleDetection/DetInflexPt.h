#pragma once
/*Detect inflexion points*/

#include<iostream>
#include<vector>
#include<opencv2/opencv.hpp>
#include<algorithm>

namespace Zikai {

	//using namespace std;
	//using namespace cv;
	struct InflexionPt
	{
		std::vector<std::vector<cv::Point> > new_edgeList;
		std::vector<std::vector<cv::Point> > new_segList;
	};

	InflexionPt detectInflexPt(
		const std::vector<std::vector<cv::Point>>& edgeList,
		const std::vector<std::vector<cv::Point>>& segList
	);


}//endnamespace Zikai