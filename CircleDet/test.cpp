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

void test();

int main()
{
	test();
	return 0;
}

void test()
{
	std::string path = "D:/circle_detection/";
	std::string dst = "D:/circle_detection_result/";
	try {
		std::filesystem::create_directory(dst);
	}
	catch (...) {}

	Zikai::CircleDetector detector;
	Zikai::CircleDetectionThreshold threshold;
	threshold.T_inlier = 0.3;
	threshold.sharp_angle = 35.0;

	detector.setThreshold(threshold);

	std::vector<std::string> filenames;
	cv::glob(path + "*.png", filenames, false);

	for (const auto& file : filenames) {
		
		cv::String::size_type pos1, pos2;
		pos1 = file.find("1");
		pos2 = file.find(".");
		cv::String prefix = file.substr(pos1 + 2, pos2 - pos1 - 2);
		cv::String suffix = file.substr(pos2 + 1, pos2 + 3);

		//the name of saved detected images
		cv::String saveName = dst + std::filesystem::path(file).stem().string() + "_det." + suffix;

		std::filesystem::path debugPath = std::filesystem::path(dst) / (std::filesystem::path(file).stem().string() + "_debug");

		detector.setEnableGaussianBlur(true);
		detector.setStepDebugDirectory(debugPath);
		
		cv::Mat inputImage = cv::imread(file, cv::IMREAD_UNCHANGED);
		auto detectedCircles = detector.detectCircles(inputImage);

		if(detectedCircles.size() > 0) {
			std::ostringstream oss;

			oss << "====================================================\n";
			oss << "File: " << std::filesystem::path(file).filename() << "\n";
			oss << "Detected circles: " << detectedCircles.size() << "\n";
			for (size_t i = 0; i < detectedCircles.size(); ++i) {
				oss << "  Circle " << i + 1 << ": (x: " << detectedCircles[i].xc
					<< ", y: " << detectedCircles[i].yc
					<< ", r: " << detectedCircles[i].r
					<< ", inlierRatio: " << detectedCircles[i].inlierRatio << ")\n";
			}
			oss << "====================================================\n";
			std::cout << oss.str();

		}

	}
	return;
}
