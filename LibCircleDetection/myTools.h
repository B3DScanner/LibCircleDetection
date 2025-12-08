#pragma once
/* myTools functions:
1. extract closed segments from the edgeLists
2. determine centers from arcs, and verify by co-circle theorem
3. fit the final arcs
4. verify the fitted circles
5. cluster the final circles
*/
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <math.h>
#include <Eigen/Dense>
#include <string>
#include <sstream>
#include <cmath>
#include <fstream>
#include <functional>
#include <algorithm>
#include <numeric>

//using namespace std;
//using namespace cv;
//using namespace Eigen;

namespace Zikai {

	// extract closed edgeLists and not closed edgeLists
	struct closedEdgesExtract {
		std::vector<std::vector<cv::Point> >closedEdges;
		std::vector<std::vector<cv::Point> >notClosedEdges;

		inline void reserve(size_t sz)
		{
			closedEdges.reserve(sz);
			notClosedEdges.reserve(sz);
		}
	};

	closedEdgesExtract extractClosedEdges(const std::vector<std::vector<cv::Point>>& edgeList);

	/*------------sort arcs by their length-------------*/
	std::vector<std::vector<cv::Point>> sortEdgeList(const std::vector<std::vector<cv::Point>>& edgelists);

	///*--------------calculate circular centers and radii--------------*/
	bool comCirCenterRadius(cv::Point A, cv::Point B, cv::Point C, double* R, cv::Point2f* O);


	int Sign(float x);

	/*-----------calculate circular centers and radii for the two arcs------------*/
	bool twoArcsCenterRadius(
		const std::vector<cv::Point>& A1B1C1,
		const std::vector<cv::Point>& A2B2C2,
		bool& flag,
		cv::Vec3f& temp1_center_radius,
		cv::Vec3f& temp2_center_radius,
		int T_o,
		int T_r
	);


	/*---------estimate centers and radius by two grouped arcs------------*/
	bool estimateCenterRadius(const std::vector<cv::Point>& A1B1C1, const std::vector<cv::Point>& A2B2C2, double& estimateR, cv::Point2f& estimateO);

	/*---------estimate centers and radius by a single arc------------*/
	bool estimateSingleCenterRadius(const std::vector<cv::Point>& A1B1C1, double& estimateR, cv::Point2f& estimateO);

	/*---------estimate centers and radius by a closed circle ------------*/
	bool estimateClosedCenterRadius(std::vector<cv::Point> A1B1C1, double* estimateR, cv::Point2f* estimateO);

	/*------------------------refine the estimated center and radius---------------------------------*/
	//first: use SVD and the default error is 0 based on Eigen library
	Eigen::MatrixXd pinv_eigen_based(Eigen::MatrixXd& origin, const float er = 1.e-6);

	// then compute refinement center and radius parameters
	cv::Vec3d refine_center_radius(std::vector<cv::Point> circle_pt, cv::Vec3f center_radius);


	/*--------------- try using co-circle theorem firstly to group arcs----------*/
	struct groupArcs {
		std::vector<std::vector<cv::Point>> arcsFromSameCircles;
		std::vector<std::vector<cv::Point>> arcsStartMidEnd;
		std::vector<cv::Vec3f> recordOR;//record the estimated centers and radii
	};


	groupArcs coCircleGroupArcs(std::vector<std::vector<cv::Point>> edgelist, int T_o, int T_r);

	/*----------circle fitting and verification-------------------*/
	// Fits a circle to a given set of points. There must be at least 2 points
	// The circle equation is of the form: (x-xc)^2 + (y-yc)^2 = r^2
	// Returns ZIKAI_TRUE if there is a fit, false in case no circles can be fit
	//
	bool CircleFit(const std::vector<double>& x, const std::vector<double>& y, int N, const std::vector<cv::Point>& stEdMid, double pxc, double pyc, double pr, double pe, double angle);




	/*----------circle verification using estimated centers and radii-------------------*/
	bool circleVerify(const std::vector<double>& x, const std::vector<double>& y, int N, const std::vector<cv::Point>& stEdMid, cv::Point2f O, double R, double& pe, double& angle);

	/*------------fit circles using grouped arcs---------*/
	struct Circles {
		double xc, yc, r, inlierRatio;
	};

	std::vector<Circles> circleFitGroupedArcs(const std::vector<std::vector<cv::Point>>& groupedArcs, const std::vector<std::vector<cv::Point>>& groupedArcsThreePt);

	/*------------fit circles using closed arcs---------*/
	std::vector<Circles> circleFitClosedArcs(const std::vector<std::vector<cv::Point>>& closedArcs);

	// cluster circles


	std::vector<Circles> clusterCircles(std::vector<Circles> totalCircles);


	/*----------verify the circles by inlier ratio----------*/
	std::vector<Circles> circleEstimateGroupedArcs(
		const std::vector<std::vector<cv::Point>>& groupedArcs,
		const std::vector<cv::Vec3f>& recordOR,
		const std::vector<std::vector<cv::Point>>& groupedArcsThreePt,
		float T_inlier,
		float T_angle
	);



	/*------------estimate circles using closed arcs---------*/
	std::vector<Circles> circleEstimateClosedArcs(std::vector<std::vector<cv::Point>> closedArcs, float T_inlier_closed);


	// draw fitted circles
	cv::Mat drawResult(bool onImage, cv::Mat srcImg, std::string srcImgName, const std::vector<Circles>& circles);

	/*-----------load ground truth txt files-------------*/

	void LoadGT(std::vector<Circles>& gt, const std::string& sGtFileName);

	/*----------compute precision, recall and F-measure------------*/
	bool TestOverlap(const cv::Mat1b& gt, const cv::Mat1b& test, float th);
	int Count(const std::vector<bool> v);

	// Should be checked !!!!!
	//  TestOverlap
	struct pre_rec_fmeasure {
		float precision;
		float recall;
		float fmeasure;
	};


	pre_rec_fmeasure Evaluate(const std::vector<Circles>& ellGT, const std::vector<Circles>& ellTest, const float th_score, const cv::Mat& img);

} //end of namespace Zikai
