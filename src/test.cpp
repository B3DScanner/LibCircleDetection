/**************************************************************************************************************
* An occlusion-resistant circle detector using inscribed triangles source code.
* Copyright (c) 2021, Mingyang Zhao
* E-mails of the authors: zhaomingyang16@mails.ucas.ac.cn
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.

* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  

* By using this implementation, please cite the following paper:
*
* M. Zhao, X. Jia, D.Y "An occlusion-resistant circle detector using inscribed triangles,"
*     Pattern Recognition (2021).
**************************************************************************************************************/

#include <iostream>
#include<opencv2/imgproc.hpp>
#include <filesystem>

#include <LibCircleDetection.h>

//using namespace cv;
//using namespace std;


/*---set thresholds---*/
// For better performance, you can mainly tune the parameters:  'T_inlier' and 'sharp_angle' 
typedef struct threshold {
	int T_l = 20;
	float T_ratio = 0.001;
	int T_o = 5;// 5 10 15 20 25
	int T_r = 5;// 5 10 15 20 25
	float T_inlier = 0.35;//0.3 0.35 0.4 0.45 0.5 (the larger the more strict)
	float T_angle = 2.0;// 
	float T_inlier_closed = 0.5;//0.5,0.6 0.7 0.8,0.9
	float sharp_angle = 60;//35 40 45 50 55 60 

}T;

int main()
{
	using namespace Zikai;

	constexpr bool STEP_DISPLAY = false; // set to ZIKAI_TRUE to display the illustration for each step
	constexpr bool USE_GAUSSIAN = ZIKAI_TRUE; // set to ZIKAI_TRUE to use Gaussian denoise (we do not use in paper)

	// You should create at least two directories: 'Images1' & 'result'
	// If you have the ground truth, you can create the directory  'GT'
	cv::String path = "E:/CircleDetection/src/";
	cv::String dst = "E:/CircleDetection/dst/";
	//cv::String GT = "D:/astudy/dataset/circle/temp/GT/";

	path = "C:/B3D/Synthetic_calibration/test_APIS_CalibrationObject3d/circle_detection/";
	dst = "C:/B3D/Synthetic_calibration/test_APIS_CalibrationObject3d/circle_detection_result/";
	try {
		std::filesystem::create_directory(dst);
	} catch(...) { }


	T test_threshold;

	test_threshold.T_inlier = 0.3;
	test_threshold.sharp_angle = 30.0;

	std::vector<cv::String> Filenames;
	cv::glob(path, Filenames);
	float fmeasureSum = 0.0;
	float precisionSum = 0.0;
	float recallSum = 0.0;
	float timeSum = 0.0;

	// Detect each image in the directory 'Images1'
	for (int i = 0; i < Filenames.size(); i++)
	{
		//read images
		cv::String file = Filenames[i];
		cv::String::size_type pos1, pos2;
		pos1 = file.find("1");
		pos2 = file.find(".");
		cv::String prefix = file.substr(pos1 + 2, pos2 - pos1 - 2);
		cv::String suffix = file.substr(pos2 + 1, pos2 + 3);

		//the name of saved detected images
		cv::String saveName = dst + prefix + "_det." + suffix;

		saveName = dst + std::filesystem::path(file).stem().string() + "_det." + suffix;

	   // Gaussian denoise (optional), we do not use in paper
		cv::Mat testImgOrigin = cv::imread(file, 1);//0:gray 1:color	
		cv::Mat testImg = testImgOrigin.clone();
		if (USE_GAUSSIAN) {
			cvtColor(testImg, testImg, cv::COLOR_BGR2GRAY);
			GaussianBlur(testImg, testImg, cv::Size(9, 9), 2, 2);
			if (STEP_DISPLAY) {
				cv::imshow("Clone Image", testImg);
				cv::waitKey();
			}
		}
		int height = testImg.rows;
		int width = testImg.cols;

		

		
		/*---------Illustration for each step---------*/
		cv::Mat test1 = cv::Mat(height, width, CV_8UC1, cv::Scalar(255));
		cv::Mat test2 = cv::Mat(height, width, CV_8UC1, cv::Scalar(255));
		cv::Mat test3 = cv::Mat(height, width, CV_8UC1, cv::Scalar(255));
		cv::Mat test4 = cv::Mat(height, width, CV_8UC1, cv::Scalar(255));
		cv::Mat test5 = cv::Mat(height, width, CV_8UC1, cv::Scalar(255));
		cv::Mat test6 = cv::Mat(height, width, CV_8UC1, cv::Scalar(255));
		cv::Mat test7 = cv::Mat(height, width, CV_8UC1, cv::Scalar(255));
		cv::Mat test8 = cv::Mat(height, width, CV_8UC1, cv::Scalar(255));
		cv::Mat test9 = cv::Mat(height, width, CV_8UC1, cv::Scalar(255));
		cv::Mat test10 = cv::Mat(height, width, CV_8UC1, cv::Scalar(255));
		cv::Mat test11 = cv::Mat(height, width, CV_8UC1, cv::Scalar(255));


		// EDPF Parameter-free Edge Segment Detection 
		clock_t start, finish;
		start = clock();
		
		EDPF testEDPF(testImg);
		cv::Mat edgePFImage = testEDPF.getEdgeImage();
		cv::Mat edge = edgePFImage.clone();
		edge = edge * -1 + 255;
		if (STEP_DISPLAY) {
			cv::imshow("Edge Image Parameter Free", edge);
			//imwrite("D:/astudy/dataset/circle/temp/result/edge.jpg", edge);
			cv::waitKey();
		}
		std::vector<std::vector<cv::Point> >EDPFsegments = testEDPF.getSegments();// get edge segments

		if (STEP_DISPLAY) {
			//plot edge images
			cvtColor(test10, test10, cv::COLOR_GRAY2BGR);
			for (int es1 = 0; es1 < EDPFsegments.size(); es1++)
			{
				int r = rand() % 256;
				int g = rand() % 256;
				int b = rand() % 256;
				cv::Scalar SegEdgesColor = cv::Scalar(b, g, r);
				for (int es2 = 0; es2 < EDPFsegments[es1].size() - 1; es2++)
				{
					cv::line(test10, EDPFsegments[es1][es2], EDPFsegments[es1][es2 + 1], SegEdgesColor, 2);//cv::Scalar(0, 0, 0)
				}
			}

			imshow("Edge Segments image", test10);
			cv::waitKey();
		}
		//imwrite("D:/Test/temp/result/edge_segment.jpg", test10);*/


		/*--------delete edge segments whose pixel number is less than 16-------------*/
		std::vector<std::vector<cv::Point>> edgeList;
		for (int i = 0; i < EDPFsegments.size(); i++)
		{
			if (EDPFsegments[i].size() >= 16)// segments should have at least 16 pixels
			{
				edgeList.push_back(EDPFsegments[i]);
			}//endif
		}//endfor


		/*----------extract closed edges-------------------*/
		//closedEdgesExtract* closedAndNotClosedEdges;
		auto closedAndNotClosedEdges = extractClosedEdges(edgeList);		
		auto& closedEdgeList = closedAndNotClosedEdges.closedEdges;

		/*--------approximate edge segments using line segments by method RDP-------*/
		std::vector<std::vector<cv::Point> > segList;
		for (int s0 = 0; s0 < edgeList.size(); s0++)
		{
			std::vector<cv::Point> segTemp;
			RamerDouglasPeucker(edgeList[s0], 2.5, segTemp);//3.0
			segList.push_back(segTemp);
		}

		/*-------------reject sharp turn angles---------------*/	
		auto newSegEdgeList = rejectSharpTurn(edgeList, segList, test_threshold.sharp_angle);
		
		//new seglist and edgelist
		const auto& newSegList  = newSegEdgeList.new_segList;
		const auto& newEdgeList = newSegEdgeList.new_edgeList;

		if (STEP_DISPLAY) {
			//plot segLists after sharp turn splitting
			cvtColor(test2, test2, cv::COLOR_GRAY2BGR);
			for (int j = 0; j < newSegList.size(); j++)
			{
				int r = rand() % 256;
				int g = rand() % 256;
				int b = rand() % 256;
				cv::Scalar colorSharpTurn = cv::Scalar(b, g, r);

				for (int jj2 = 0; jj2 < newEdgeList[j].size() - 1; jj2++)
				{
					//circle(test2, newSegList[j][jj], 1, cv::Scalar(0, 0, 0), 3);
					line(test2, newEdgeList[j][jj2], newEdgeList[j][jj2 + 1], colorSharpTurn, 2);
				}

			}

			imshow("After sharp turn", test2);
			cv::waitKey();
		}

		//imwrite("sharpTurn.jpg", test2);



		/*-----------------Detect inflexion points--------------*/

		auto newSegEdgeListAfterInflexion = detectInflexPt(newEdgeList, newSegList);
		
		// new seglist and edgelist
		auto& newSegListAfterInflexion = newSegEdgeListAfterInflexion.new_segList;
		auto& newEdgeListAfterInfexion = newSegEdgeListAfterInflexion.new_edgeList;


		/*--------delete short edgeLists or near line segments----------*/
		std::vector<std::vector<cv::Point>>::iterator it = newEdgeListAfterInfexion.begin();
		while (it != newEdgeListAfterInfexion.end())
		{
			/*compute the line segment generated by the two endpoints of the arc,
			and then judge the midpoint of the arc if lying on or near the line
			*/
			cv::Point edgeSt = cv::Point((*it).front().x, (*it).front().y);
			cv::Point edgeEd = cv::Point((*it).back().x, (*it).back().y);
			int midIndex = (*it).size() / 2;

			cv::Point edgeMid = cv::Point((*it)[midIndex].x, (*it)[midIndex].y);

			double distStEd = sqrt(pow(edgeSt.x - edgeEd.x, 2) + pow(edgeSt.y - edgeEd.y, 2));
			double distStMid = sqrt(pow(edgeSt.x - edgeMid.x, 2) + pow(edgeSt.y - edgeMid.y, 2));
			double distMidEd = sqrt(pow(edgeEd.x - edgeMid.x, 2) + pow(edgeEd.y - edgeMid.y, 2));
			double distDifference = abs((distStMid + distMidEd) - distStEd);


			if ((*it).size() <= test_threshold.T_l || distDifference <= test_threshold.T_ratio * (distStMid + distMidEd))// 2 3 fixed number; (*it).size() <=20
			{
				it = newEdgeListAfterInfexion.erase(it);
			}
			else { it++; }
		}//endwhile

		if (STEP_DISPLAY) {
			cvtColor(test11, test11, cv::COLOR_GRAY2BGR);
			for (int j = 0; j < newEdgeListAfterInfexion.size(); j++)
			{
				int r = rand() % 256;
				int g = rand() % 256;
				int b = rand() % 256;
				cv::Scalar colorAfterDeleteLinePt = cv::Scalar(b, g, r);
				for (int jj2 = 0; jj2 < newEdgeListAfterInfexion[j].size() - 1; jj2++)
				{
					//circle(test2, newSegList[j][jj], 1, cv::Scalar(0, 0, 0), 3);
					line(test11, newEdgeListAfterInfexion[j][jj2], newEdgeListAfterInfexion[j][jj2 + 1], colorAfterDeleteLinePt, 2);
				}
			}

			imshow("After short and line segments remove", test11);
			cv::waitKey();
		}
		//imwrite("D:/Test/temp/result/remove_short_line.jpg", test3);


		/*-----extract closed edgeLists and not closed edgeLists after inflexion point operation------*/
	
		auto closedAndNotClosedEdges1 = extractClosedEdges(newEdgeListAfterInfexion);
		const auto& closedEdgeList1 = closedAndNotClosedEdges1.closedEdges;
		const auto& notclosedEdgeList1 = closedAndNotClosedEdges1.notClosedEdges;

		if (STEP_DISPLAY) {
			//plot closed edgeLists
			cvtColor(test4, test4, cv::COLOR_GRAY2BGR);
			for (int j = 0; j < closedEdgeList1.size(); j++)
			{
				int r = rand() % 256;
				int g = rand() % 256;
				int b = rand() % 256;
				cv::Scalar colorClosedEdges = cv::Scalar(b, g, r);
				for (int jj = 0; jj < closedEdgeList1[j].size() - 1; jj++)
				{
					//circle(test4, newSegListAfterInflexion[j][jj], 1, cv::Scalar(0, 0, 0), 3);
					cv::line(test4, closedEdgeList1[j][jj], closedEdgeList1[j][jj + 1], colorClosedEdges, 2);
				}
				//imshow("After infexion point remove", test2);
				//cv::waitKey()
			}
			imshow("closedEdges2", test4);
			cv::waitKey();
		}
		//imwrite("closedEdges2.jpg", test4);

		/*----------sort notclosedEdgeList for grouping-------------*/
		std::vector<std::vector<cv::Point>> sortedEdgeList = sortEdgeList(notclosedEdgeList1);

		/*--------------group sortededgeList---------------*/
		auto arcs = coCircleGroupArcs(sortedEdgeList, test_threshold.T_o, test_threshold.T_r);
		const auto& groupedArcs = arcs.arcsFromSameCircles;
		const auto& groupedArcsThreePt = arcs.arcsStartMidEnd;
		std::vector<cv::Vec3f>  groupedOR = arcs.recordOR;


		/*--------circle verification using estimated center and radius parameters*/
		std::vector<Circles> groupedCircles;// grouped arcs
		groupedCircles = circleEstimateGroupedArcs(groupedArcs, groupedOR, groupedArcsThreePt, test_threshold.T_inlier, test_threshold.T_angle);//fit grouped arcs

		// closed arcs
		for (auto ite = closedEdgeList1.begin(); ite != closedEdgeList1.end(); ite++)
		{
			closedEdgeList.push_back(*ite);
		}//endfor


		std::vector<Circles> closedCircles;// closedCircles
		closedCircles = circleEstimateClosedArcs(closedEdgeList, test_threshold.T_inlier_closed);// fit closed edges


		//put grouped and closed circles together
		std::vector<Circles> totalCircles;
		if (!groupedCircles.empty())
		{
			totalCircles = groupedCircles;
		}
		if (!closedCircles.empty())
		{
			for (auto it = closedCircles.begin(); it != closedCircles.end(); it++)
			{
				totalCircles.push_back(*it);
			}
		}
		//cluster circles----------------->no clustering 
		finish = clock();
		std::vector<Circles> preCircles;
		preCircles = clusterCircles(totalCircles);
		//finish = clock();
		timeSum += ((float)(finish - start) / CLOCKS_PER_SEC);
		//draw fit circles after clustering
		cv::Mat detectCircles = drawResult(ZIKAI_TRUE, testImgOrigin, saveName, preCircles);//totalCircles preCircles
	//}//endfor    run 100 times and then calculate the average


	
		/*-----compute precision, recall and fmeasure-------*/
		//pre_rec_fmeasure totalResult = Evaluate(gt, preCircles, 0.8f, testImg);
		//cv::waitKey()
		////fmeasureSum += totalResult.fmeasure;
		//precisionSum += totalResult.precision;
		//recallSum += totalResult.recall;

	}

	float avePre = precisionSum / Filenames.size();//Filenames.size()
	float aveRec = recallSum / Filenames.size();//Filenames.size()
	float aveTime = timeSum / Filenames.size();
	float aveFmea = 2 * avePre * aveRec / (avePre + aveRec);
	std::cout << "Pre Rec Fmea Time: " << avePre << " " << aveRec << " " << aveFmea << " " << aveTime << std::endl;
	cv::waitKey(0);

	return 0;
}



