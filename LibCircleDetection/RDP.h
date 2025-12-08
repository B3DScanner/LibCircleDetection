#pragma once
//2D implementation of the Ramer-Douglas-Peucker algorithm
//By Tim Sheerman-Chase, 2016
//Released under CC0
//https://en.wikipedia.org/wiki/Ramer%E2%80%93Douglas%E2%80%93Peucker_algorithm

#ifndef  _RDP_
#define _RDP_

#include <iostream>
#include <cmath>
#include <utility>
#include <vector>
#include <stdexcept>
#include<opencv2/opencv.hpp>

//using namespace std;
//using namespace cv;

//typedef std::pair<double, double> cv::Point;//typedef std::pair<double, double> cv::Point;

namespace Zikai
{
	double PerpendicularDistance(const cv::Point& pt, const cv::Point& lineStart, const cv::Point& lineEnd);

	void RamerDouglasPeucker(const std::vector<cv::Point>& pointList, double epsilon, std::vector<cv::Point>& out); // const std::vector<cv::Point> &pointList
	
}
#endif