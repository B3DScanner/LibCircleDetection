#include "myTools.h"
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
#include <numeric>



Zikai::closedEdgesExtract Zikai::extractClosedEdges(const std::vector<std::vector<cv::Point>>& edgeList)
{
	closedEdgesExtract edges;
	edges.reserve(edgeList.size());
	for (int i = 0; i < (edgeList).size(); i++)
	{
		const auto& st = edgeList[i].front();
		const auto& ed = edgeList[i].back();

		double dist = sqrt(pow(st.x - ed.x, 2) + pow(st.y - ed.y, 2));
		if (dist <= 3)
		{
			edges.closedEdges.push_back(edgeList[i]);
		}
		else
		{
			edges.notClosedEdges.push_back(edgeList[i]);
		}
	}
	return edges;
}

/*------------sort arcs by their length-------------*/
std::vector<std::vector<cv::Point>> Zikai::sortEdgeList(const std::vector<std::vector<cv::Point>>& edgelists)
{
	std::vector<int> indices(edgelists.size());
	std::iota(indices.begin(), indices.end(), 0);

	std::sort(indices.begin(), indices.end(),
		[&edgelists](int i1, int i2) {return edgelists[i1].size() > edgelists[i2].size(); });

	std::vector<std::vector<cv::Point>> results(edgelists.size());
	for (size_t i = 0; i < indices.size(); ++i) {
		results[i] = edgelists[indices[i]];
	}

	return results;
}




///*--------------calculate circular centers and radii--------------*/
bool Zikai::comCirCenterRadius(cv::Point A, cv::Point B, cv::Point C, double* R, cv::Point2f* O)
{

	//the length for triangle edges
	double AB = sqrt(pow(A.x - B.x, 2) + pow(A.y - B.y, 2));
	double CB = sqrt(pow(C.x - B.x, 2) + pow(C.y - B.y, 2));
	double AC = sqrt(pow(A.x - C.x, 2) + pow(A.y - C.y, 2));
	// the areas of the two triangles
	double p = (AB + CB + AC) / 2;
	double S_ABC = sqrt(p * (p - AB) * (p - CB) * (p - AC));// Hallen formulation 
	// radius from areas
	*R = (AB * CB * AC) / (4 * S_ABC);

	//calculate the circular center based on the triangles' vertices
	// for ABC
	double a11 = 2 * (B.y - A.y);
	double b11 = 2 * (B.x - A.x);
	double c11 = pow(B.y, 2) + pow(B.x, 2) - pow(A.y, 2) - pow(A.x, 2);
	double a12 = 2 * (C.y - B.y);
	double b12 = 2 * (C.x - B.x);
	double c12 = pow(C.y, 2) + pow(C.x, 2) - pow(B.y, 2) - pow(B.x, 2);
	(*O).x = (a11 * c12 - a12 * c11) / (a11 * b12 - a12 * b11);
	(*O).y = (c11 * b12 - c12 * b11) / (a11 * b12 - a12 * b11);
	return true;

}


int Zikai::Sign(float x)
{
	int sign;
	if (x > 0)
		return 1;
	else
		return -1;

}

/*-----------calculate circular centers and radii for the two arcs------------*/
bool Zikai::twoArcsCenterRadius(const std::vector<cv::Point>& A1B1C1, const std::vector<cv::Point>& A2B2C2, bool& flag, cv::Vec3f& temp1_center_radius, cv::Vec3f& temp2_center_radius, int T_o, int T_r)
{

	flag = false;
	// constraint 1:
	//S2 E2 and M1 lie in different sizes of Line S1E1 && S1 E1 and M2 lie in different sizes of Line S2E2
	cv::Point S1 = A1B1C1[2];//A1B1C1.front()
	cv::Point E1 = *(A1B1C1.end() - 2);//A1B1C1.back()
	cv::Point S2 = A2B2C2[2];//A2B2C2.front()
	cv::Point E2 = *(A2B2C2.end() - 2);//A2B2C2.back()
	cv::Point M1 = A1B1C1[A1B1C1.size() / 2];
	cv::Point M2 = A2B2C2[A2B2C2.size() / 2];
	// Line S1E1, S2E2
	float K_S1E1 = (S1.x - E1.x) / (S1.y - E1.y + 1e-6);
	float K_S2E2 = (S2.x - E2.x) / (S2.y - E2.y + 1e-6);
	// the location of midpoint M1 regarding line S1E1, M2 regarding line S2E2
	int SignM1 = Sign(M1.x - S1.x - K_S1E1 * (M1.y - S1.y));
	int SignM2 = Sign(M2.x - S2.x - K_S2E2 * (M2.y - S2.y));
	// S2, E2 of A2B2C2 should lie in different sides of M1 && S1, E1 of A1B1C1 should lie in different sides of M2
	int SignS2 = Sign(S2.x - S1.x - K_S1E1 * (S2.y - S1.y));
	int SignE2 = Sign(E2.x - S1.x - K_S1E1 * (E2.y - S1.y));
	int SignS1 = Sign(S1.x - S2.x - K_S2E2 * (S1.y - S2.y));
	int SignE1 = Sign(E1.x - S2.x - K_S2E2 * (E1.y - S2.y));
	if (SignS1 * SignE1 >= 0 || SignS2 * SignE2 >= 0)// different sides then check constraint 2
	{
		//if constratint 1 holds, then check constratint 2
		cv::Point A1 = A1B1C1[5];//A1B1C1.front()  A1B1C1[5]
		cv::Point C1 = *(A1B1C1.end() - 6);//A1B1C1.back() *(A1B1C1.end()-6)
		int firstMidIndex = A1B1C1.size() / 2;
		cv::Point B1 = A1B1C1[firstMidIndex];


		cv::Point A2 = *(A2B2C2.begin() + 5);//A2B2C2.front()
		cv::Point C2 = *(A2B2C2.end() - 6);//A2B2C2.back()
		int secondMidIndex = A2B2C2.size() / 2;
		cv::Point B2 = A2B2C2[secondMidIndex];


		// the length of edge A1C1 
		double A1C1 = sqrt(pow(A1.x - C1.x, 2) + pow(A1.y - C1.y, 2));
		double A2C2 = sqrt(pow(A2.x - C2.x, 2) + pow(A2.y - C2.y, 2));
		// use the co-circle theorem to verify if the first arc and the second arc are 
		// from the same circle
		//the length for triangle edges
		double A1B2 = sqrt(pow(A1.x - B2.x, 2) + pow(A1.y - B2.y, 2));
		double C1B2 = sqrt(pow(C1.x - B2.x, 2) + pow(C1.y - B2.y, 2));
		double A2B1 = sqrt(pow(A2.x - B1.x, 2) + pow(A2.y - B1.y, 2));
		double C2B1 = sqrt(pow(C2.x - B1.x, 2) + pow(C2.y - B1.y, 2));

		// the areas of the two triangles
		double p1 = (A1B2 + C1B2 + A1C1) / 2;
		double S_A1B2C1 = sqrt(p1 * (p1 - A1B2) * (p1 - C1B2) * (p1 - A1C1));// Hallen formulation 
		double p2 = (A2B1 + C2B1 + A2C2) / 2;
		double S_A2B1C2 = sqrt(p2 * (p2 - A2B1) * (p2 - C2B1) * (p2 - A2C2));// Hallen formulation 

		// use the ratio of the multiplication of edge length to area

		double R1 = (A1B2 * C1B2 * A1C1) / (4 * S_A1B2C1);
		double R2 = (A2B1 * C2B1 * A2C2) / (4 * S_A2B1C2);

		if (abs(R1 - R2) <= T_r)//less than some certain threshold->group
		{
			//calculate the circumcircle center from the triangles' vertices 

			// for A1B2C1
			double a11 = 2 * (B2.y - A1.y);
			double b11 = 2 * (B2.x - A1.x);
			double c11 = pow(B2.y, 2) + pow(B2.x, 2) - pow(A1.y, 2) - pow(A1.x, 2);
			double a12 = 2 * (C1.y - B2.y);
			double b12 = 2 * (C1.x - B2.x);
			double c12 = pow(C1.y, 2) + pow(C1.x, 2) - pow(B2.y, 2) - pow(B2.x, 2);

			//for A2B1C2
			double a21 = 2 * (B1.y - A2.y);
			double b21 = 2 * (B1.x - A2.x);
			double c21 = pow(B1.y, 2) + pow(B1.x, 2) - pow(A2.y, 2) - pow(A2.x, 2);
			double a22 = 2 * (C2.y - B1.y);
			double b22 = 2 * (C2.x - B1.x);
			double c22 = pow(C2.y, 2) + pow(C2.x, 2) - pow(B1.y, 2) - pow(B1.x, 2);

			//centers
			cv::Point2f O1, O2;
			O1.x = (a11 * c12 - a12 * c11) / (a11 * b12 - a12 * b11);
			O1.y = (c11 * b12 - c12 * b11) / (a11 * b12 - a12 * b11);

			O2.x = (a21 * c22 - a22 * c21) / (a21 * b22 - a22 * b21);
			O2.y = (c21 * b22 - c22 * b21) / (a21 * b22 - a22 * b21);

			double distO1O2 = sqrt(pow(O1.y - O2.y, 2) + pow(O1.x - O2.x, 2));

			if (distO1O2 <= T_o)
			{

				// compute inlier ratio
				double R = (R1 + R2) / 2;
				cv::Point2f O;
				O.x = (O1.x + O2.x) / 2;
				O.y = (O1.y + O2.y) / 2;

				int num1 = 0, num2 = 0;
				double dis1, dis2;
				for (auto it1 = A1B1C1.begin(); it1 != A1B1C1.end(); it1++)
				{
					dis1 = sqrt(pow((*it1).x - O.x, 2) + pow((*it1).y - O.y, 2));
					if (abs(dis1 - R) <= 2)
						num1++;
				}//endfor

				for (auto it2 = A2B2C2.begin(); it2 != A2B2C2.end(); it2++)
				{
					dis2 = sqrt(pow((*it2).x - O.x, 2) + pow((*it2).y - O.y, 2));
					if (abs(dis2 - R) <= 2)
						num2++;
				}//endfor
				int size1 = A1B1C1.size();
				int size2 = A2B2C2.size();
				double edgeInlier = (double)(num1 + num2) / (size1 + size2);

				if (edgeInlier >= 0.2)//0.4
				{
					flag = true;
					temp1_center_radius[0] = O1.x;
					temp1_center_radius[1] = O1.y;
					temp1_center_radius[2] = R1;
					temp2_center_radius[0] = O2.x;
					temp2_center_radius[1] = O2.y;
					temp2_center_radius[2] = R2;
				}
			}//endif

		}
	}

	return true;
}


/*---------estimate centers and radius by two grouped arcs------------*/
bool Zikai::estimateCenterRadius(const std::vector<cv::Point>& A1B1C1, const std::vector<cv::Point>& A2B2C2, double& estimateR, cv::Point2f& estimateO)
{

	cv::Point A1 = A1B1C1[5];//A1B1C1.front()
	cv::Point C1 = *(A1B1C1.end() - 6);//A1B1C1.back()
	int firstMidIndex = A1B1C1.size() / 2;
	cv::Point B1 = A1B1C1[firstMidIndex];


	cv::Point A2 = *(A2B2C2.begin() + 5);//A2B2C2.front()
	cv::Point C2 = *(A2B2C2.end() - 6);//A2B2C2.back()
	int secondMidIndex = A2B2C2.size() / 2;
	cv::Point B2 = A2B2C2[secondMidIndex];




	// the length of edge A1C1 
	double A1C1 = sqrt(pow(A1.x - C1.x, 2) + pow(A1.y - C1.y, 2));
	double A2C2 = sqrt(pow(A2.x - C2.x, 2) + pow(A2.y - C2.y, 2));

	//the length for triangle edges
	double A1B2 = sqrt(pow(A1.x - B2.x, 2) + pow(A1.y - B2.y, 2));
	double C1B2 = sqrt(pow(C1.x - B2.x, 2) + pow(C1.y - B2.y, 2));
	double A2B1 = sqrt(pow(A2.x - B1.x, 2) + pow(A2.y - B1.y, 2));
	double C2B1 = sqrt(pow(C2.x - B1.x, 2) + pow(C2.y - B1.y, 2));

	// the areas of the two triangles
	double p1 = (A1B2 + C1B2 + A1C1) / 2;
	double S_A1B2C1 = sqrt(p1 * (p1 - A1B2) * (p1 - C1B2) * (p1 - A1C1));// Hallen formulation 
	double p2 = (A2B1 + C2B1 + A2C2) / 2;
	double S_A2B1C2 = sqrt(p2 * (p2 - A2B1) * (p2 - C2B1) * (p2 - A2C2));// Hallen formulation 

	// use the ratio of the multiplication of edge length to area

	double R1 = (A1B2 * C1B2 * A1C1) / (4 * S_A1B2C1);
	double R2 = (A2B1 * C2B1 * A2C2) / (4 * S_A2B1C2);

	//calculate the circumcircle center from the triangles' vertices
	// for A1B2C1
	double a11 = 2 * (B2.y - A1.y);
	double b11 = 2 * (B2.x - A1.x);
	double c11 = pow(B2.y, 2) + pow(B2.x, 2) - pow(A1.y, 2) - pow(A1.x, 2);
	double a12 = 2 * (C1.y - B2.y);
	double b12 = 2 * (C1.x - B2.x);
	double c12 = pow(C1.y, 2) + pow(C1.x, 2) - pow(B2.y, 2) - pow(B2.x, 2);

	//for A2B1C2
	double a21 = 2 * (B1.y - A2.y);
	double b21 = 2 * (B1.x - A2.x);
	double c21 = pow(B1.y, 2) + pow(B1.x, 2) - pow(A2.y, 2) - pow(A2.x, 2);
	double a22 = 2 * (C2.y - B1.y);
	double b22 = 2 * (C2.x - B1.x);
	double c22 = pow(C2.y, 2) + pow(C2.x, 2) - pow(B1.y, 2) - pow(B1.x, 2);
	//centers
	cv::Point2f O1, O2;
	O1.x = (a11 * c12 - a12 * c11) / (a11 * b12 - a12 * b11);
	O1.y = (c11 * b12 - c12 * b11) / (a11 * b12 - a12 * b11);

	O2.x = (a21 * c22 - a22 * c21) / (a21 * b22 - a22 * b21);
	O2.y = (c21 * b22 - c22 * b21) / (a21 * b22 - a22 * b21);

	// S_A1C1A2
	double R_A1C1A2;
	cv::Point2f O_A1C1A2;
	comCirCenterRadius(A1, C1, A2, &R_A1C1A2, &O_A1C1A2);

	// S_A1C1C2
	double R_A1C1C2;
	cv::Point2f O_A1C1C2;
	comCirCenterRadius(A1, C1, C2, &R_A1C1C2, &O_A1C1C2);

	// S_A2C2A1
	double R_A2C2A1;
	cv::Point2f O_A2C2A1;
	comCirCenterRadius(A2, C2, A1, &R_A2C2A1, &O_A2C2A1);

	// S_A2C2C1
	double R_A2C2C1;
	cv::Point2f O_A2C2C1;
	comCirCenterRadius(A2, C2, C1, &R_A2C2C1, &O_A2C2C1);

	//using median computes centers and radius
	std::vector<double> tempR, tempOX, tempOY;
	// R 
	tempR.push_back(R1);
	tempR.push_back(R2);
	tempR.push_back(R_A1C1A2);
	tempR.push_back(R_A1C1C2);
	tempR.push_back(R_A2C2A1);
	tempR.push_back(R_A2C2C1);

	sort(tempR.begin(), tempR.end());
	estimateR = (tempR[2] + tempR[3]) / 2.0;
	// O.x
	tempOX.push_back(O1.x);
	tempOX.push_back(O2.x);
	tempOX.push_back(O_A1C1A2.x);
	tempOX.push_back(O_A1C1C2.x);
	tempOX.push_back(O_A2C2A1.x);
	tempOX.push_back(O_A2C2C1.x);

	sort(tempOX.begin(), tempOX.end());
	double estimateOX = (tempOX[2] + tempOX[3]) / 2.0;
	//O.y
	tempOY.push_back(O1.y);
	tempOY.push_back(O2.y);
	tempOY.push_back(O_A1C1A2.y);
	tempOY.push_back(O_A1C1C2.y);
	tempOY.push_back(O_A2C2A1.y);
	tempOY.push_back(O_A2C2C1.y);

	sort(tempOY.begin(), tempOY.end());
	double estimateOY = (tempOY[2] + tempOY[3]) / 2.0;
	estimateO = cv::Point2f(estimateOX, estimateOY);
	return true;

}

/*---------estimate centers and radius by a single arc------------*/
bool Zikai::estimateSingleCenterRadius(const std::vector<cv::Point>& A1B1C1, double& estimateR, cv::Point2f& estimateO)
{
	cv::Point A1 = A1B1C1[5];//A1B1C1.front()
	cv::Point C1 = *(A1B1C1.end() - 6);//A1B1C1.back()
	int firstMidIndex = A1B1C1.size() / 2;
	cv::Point B1 = A1B1C1[firstMidIndex];
	cv::Point D11 = A1B1C1[5 + (firstMidIndex - 5) / 2];
	cv::Point D12 = A1B1C1[firstMidIndex + (A1B1C1.size() - firstMidIndex) / 2];


	// first estimate center and radius using arcs by themselves

	// for arc A1B1C1
	// S_A1B1C1
	double R_A1B1C1;
	cv::Point2f O_A1B1C1;
	comCirCenterRadius(A1, B1, C1, &R_A1B1C1, &O_A1B1C1);

	// S_A1D11C1
	double R_A1D11C1;
	cv::Point2f O_A1D11C1;
	comCirCenterRadius(A1, D11, C1, &R_A1D11C1, &O_A1D11C1);

	// S_A1D12C1
	double R_A1D12C1;
	cv::Point2f O_A1D12C1;
	comCirCenterRadius(A1, D12, C1, &R_A1D12C1, &O_A1D12C1);


	// using median computes centers and radius
	std::vector<double> tempR, tempOX, tempOY;
	// R 
	tempR.push_back(R_A1B1C1);
	tempR.push_back(R_A1D11C1);
	tempR.push_back(R_A1D12C1);
	//OX
	tempOX.push_back(O_A1B1C1.x);
	tempOX.push_back(O_A1D11C1.x);
	tempOX.push_back(O_A1D12C1.x);
	//OY
	tempOY.push_back(O_A1B1C1.y);
	tempOY.push_back(O_A1D11C1.y);
	tempOY.push_back(O_A1D12C1.y);

	sort(tempR.begin(), tempR.end());
	estimateR = tempR[1];

	sort(tempOX.begin(), tempOX.end());
	double estimateOX = tempOX[1];

	sort(tempOY.begin(), tempOY.end());
	double estimateOY = tempOY[1];
	estimateO = cv::Point2f(estimateOX, estimateOY);
	return true;

}


/*---------estimate centers and radius by a closed circle ------------*/
bool Zikai::estimateClosedCenterRadius(std::vector<cv::Point> A1B1C1, double* estimateR, cv::Point2f* estimateO)
{
	// sample 5pts evenly
	int ptNum = A1B1C1.size();
	cv::Point A = A1B1C1[ptNum / 5];
	cv::Point B = A1B1C1[2 * ptNum / 5];
	cv::Point C = A1B1C1[3 * ptNum / 5];
	cv::Point D = A1B1C1[4 * ptNum / 5];
	cv::Point E = A1B1C1[ptNum - 7];//A1B1C1[ptNum-10]



	// five new inscribed triangles
	//ACD
	double R_ACD;
	cv::Point2f O_ACD;
	comCirCenterRadius(A, C, D, &R_ACD, &O_ACD);

	//ACE
	double R_ACE;
	cv::Point2f O_ACE;
	comCirCenterRadius(A, C, E, &R_ACE, &O_ACE);


	//ABD
	double R_ABD;
	cv::Point2f O_ABD;
	comCirCenterRadius(A, B, D, &R_ABD, &O_ABD);


	//BCE
	double R_BCE;
	cv::Point2f O_BCE;
	comCirCenterRadius(B, C, E, &R_BCE, &O_BCE);


	//DBE
	double R_DBE;
	cv::Point2f O_DBE;
	comCirCenterRadius(D, B, E, &R_DBE, &O_DBE);


	// using median computes centers and radius
	std::vector<double> tempR, tempOX, tempOY;
	// R 
	tempR.push_back(R_ACD);
	tempR.push_back(R_ACE);
	tempR.push_back(R_ABD);
	tempR.push_back(R_BCE);
	tempR.push_back(R_DBE);
	//OX
	tempOX.push_back(O_ACD.x);
	tempOX.push_back(O_ACE.x);
	tempOX.push_back(O_ABD.x);
	tempOX.push_back(O_BCE.x);
	tempOX.push_back(O_DBE.x);

	//OY
	tempOY.push_back(O_ACD.y);
	tempOY.push_back(O_ACE.y);
	tempOY.push_back(O_ABD.y);
	tempOY.push_back(O_BCE.y);
	tempOY.push_back(O_DBE.y);


	sort(tempR.begin(), tempR.end());
	*estimateR = tempR[2];

	sort(tempOX.begin(), tempOX.end());
	double estimateOX = tempOX[2];

	sort(tempOY.begin(), tempOY.end());
	double estimateOY = tempOY[2];
	*estimateO = cv::Point2f(estimateOX, estimateOY);

	return true;

}

/*------------------------refine the estimated center and radius---------------------------------*/
//first: use SVD and the default error is 0 based on Eigen library
Eigen::MatrixXd Zikai::pinv_eigen_based(Eigen::MatrixXd& origin, const float er)
{
	// SVD
	Eigen::JacobiSVD<Eigen::MatrixXd> svd_holder(origin, Eigen::ComputeThinU | Eigen::ComputeThinV);
	// the result of SVD: UVD
	Eigen::MatrixXd U = svd_holder.matrixU();
	Eigen::MatrixXd V = svd_holder.matrixV();
	Eigen::MatrixXd D = svd_holder.singularValues();

	// construct S
	Eigen::MatrixXd S(V.cols(), U.cols());
	S.setZero();
	// inverse singularValues  and store in S
	for (unsigned int i = 0; i < D.size(); ++i)
	{

		if (D(i, 0) > er) {
			S(i, i) = 1 / D(i, 0);
		}
		else {
			S(i, i) = 0;
		}
	}

	// pinv_matrix = V * S * U^T
	return V * S * U.transpose();
}

// then compute refinement center and radius parameters
cv::Vec3d Zikai::refine_center_radius(std::vector<cv::Point> circle_pt, cv::Vec3f center_radius)
{
	int pt_num = circle_pt.size();
	Eigen::MatrixXd A(pt_num, 3);
	Eigen::MatrixXd E(pt_num, 1);
	cv::Point2f circle_center(center_radius[0], center_radius[1]);
	double circle_r = center_radius[2];
	// construct coefficient matrix A and value matrix E
	for (int i = 0; i < pt_num; i++)
	{
		A(i, 0) = circle_pt[i].x - circle_center.x;
		A(i, 1) = circle_pt[i].y - circle_center.y;
		A(i, 2) = circle_r;
		E(i) = pow(A(i, 0), 2) + pow(A(i, 1), 2) - pow(A(i, 2), 2);
	}
	// pseudoinverse of A: use SVD and default error is 0
	cv::Vec3d center_radius_refinement;
	center_radius_refinement[0] = center_radius[0] - (-1 / 2.0 * pinv_eigen_based(A) * E)(0);
	center_radius_refinement[1] = center_radius[1] - (-1 / 2.0 * pinv_eigen_based(A) * E)(1);
	center_radius_refinement[2] = center_radius[2] - (-1 / 2.0 * pinv_eigen_based(A) * E)(2);

	return center_radius_refinement;
}



Zikai::groupArcs Zikai::coCircleGroupArcs(std::vector<std::vector<cv::Point>> edgelist, int T_o, int T_r)
{
	groupArcs arcs;

	// use to empty some edgelist
	std::vector<cv::Point> vec;
	cv::Point temp = cv::Point(0, 0);
	vec.push_back(temp);

	for (int i = 0; i < edgelist.size(); i++)
	{
		int leng = edgelist[i].size();
		if (leng == 1) { continue; }
		// put edge1 into outEdgeList first
		std::vector<cv::Point> CirPt;
		std::vector<std::vector<cv::Point> >outEdgeList;
		outEdgeList.push_back(edgelist[i]);

		cv::Vec3f groupedOR;
		std::vector<cv::Point> outThreePt;
		cv::Point start = edgelist[i].front();
		cv::Point end = edgelist[i].back();
		cv::Point mid = edgelist[i][leng / 2];
		outThreePt.push_back(start);
		outThreePt.push_back(end);
		outThreePt.push_back(mid);

		std::vector<cv::Vec3f> CenterRadius;// record center and radius for every paired arcs, used for further radii and center estimation 
		//iterate to find the grouping arcs
		for (int j = 0; j < edgelist.size(); j++)
		{
			if (j == i || edgelist[j].size() == 1) { continue; }
			else
			{
				// group edgelist[j] with outEdgeList
				bool flag = false;
				int pass = 0;// record whether two edges can be grouped
				cv::Vec3f temp1_center_radius;// record the two radii and centers for paired two arcs
				cv::Vec3f temp2_center_radius;// temp1,temp2
				for (int k = 0; k < outEdgeList.size(); k++)
				{
					twoArcsCenterRadius(outEdgeList[k], edgelist[j], flag, temp1_center_radius, temp2_center_radius, T_o, T_r);
					if (flag)//can be grouped  
					{
						pass++;
						CenterRadius.push_back(temp1_center_radius);// record two radii and centers for paired arcs
						CenterRadius.push_back(temp2_center_radius);
					}
				}//endfor
				if (pass == outEdgeList.size())// edgelist[j] can be grouped with  all before outEdgeList
				{
					outEdgeList.push_back(edgelist[j]);
					cv::Point start2 = edgelist[j].front();
					cv::Point end2 = edgelist[j].back();
					cv::Point mid2 = edgelist[j][edgelist[j].size() / 2];
					outThreePt.push_back(start2);
					outThreePt.push_back(end2);
					outThreePt.push_back(mid2);
					edgelist[j] = vec;
				}//endif
			}//endelse
		}//inner endfor

		// put points together 
		for (int l1 = 0; l1 < outEdgeList.size(); l1++)
		{
			for (int l2 = 0; l2 < outEdgeList[l1].size(); l2++)
			{
				CirPt.push_back(outEdgeList[l1][l2]);
			}
		}
		// estimate center and radius for single arc
		if (outEdgeList.size() == 1)// edgelist[i] didn't find grouped arcs
		{
			double singleR;
			cv::Point2f singleO;
			estimateSingleCenterRadius(edgelist[i], singleR, singleO);
			groupedOR = cv::Vec3f(singleO.x, singleO.y, singleR);
		}
		// estimate center and radius for two arcs
		if (outEdgeList.size() == 2)// edgelist[i] finds one arc to pair
		{
			double TwoR;
			cv::Point2f TwoO;
			estimateCenterRadius(outEdgeList[0], outEdgeList[1], TwoR, TwoO);
			groupedOR = cv::Vec3f(TwoO.x, TwoO.y, TwoR);
		}

		//estimate center and radius for three or more arcs
		if (outEdgeList.size() > 2)// edgelist[i] finds more than two arcs, then take the median as estimated value
		{
			std::vector<double> temp_r;
			std::vector<float> temp_x;
			std::vector<float> temp_y;
			for (int i = 0; i < CenterRadius.size(); i++)
			{
				temp_x.push_back(CenterRadius[i][0]);
				temp_y.push_back(CenterRadius[i][1]);
				temp_r.push_back(CenterRadius[i][2]);
			}
			sort(temp_r.begin(), temp_r.end());
			sort(temp_x.begin(), temp_x.end());
			sort(temp_y.begin(), temp_y.end());
			// find the median for more than two arcs
			double MoreR;
			cv::Point2f MoreO;
			int num_center_radius = CenterRadius.size();
			if (num_center_radius / 2 == 0)// even number
			{
				MoreR = (temp_r[num_center_radius / 2] + temp_r[(num_center_radius / 2) + 1]) / 2.0;
				MoreO.x = (temp_x[num_center_radius / 2] + temp_x[(num_center_radius / 2) + 1]) / 2;
				MoreO.y = (temp_y[num_center_radius / 2] + temp_y[(num_center_radius / 2) + 1]) / 2;

			}
			else {
				MoreR = temp_r[(num_center_radius + 1) / 2.0];
				MoreO.x = temp_x[(num_center_radius + 1) / 2.0];
				MoreO.y = temp_y[(num_center_radius + 1) / 2.0];

			}

			groupedOR = cv::Vec3f(MoreO.x, MoreO.y, MoreR);
		}

		// refine the estimated center and radius
		cv::Vec3d center_radius_refinement = refine_center_radius(CirPt, groupedOR);

		// finally detected grouped arcs, three points on it, and recoreded center and radius. 
		edgelist[i] = vec;
		arcs.arcsFromSameCircles.push_back(CirPt);
		arcs.arcsStartMidEnd.push_back(outThreePt);
		arcs.recordOR.push_back(center_radius_refinement);
	}//endfor outer
	return arcs;
}


/*----------circle fitting and verification-------------------*/
// Fits a circle to a given set of points. There must be at least 2 points
// The circle equation is of the form: (x-xc)^2 + (y-yc)^2 = r^2
// Returns ZIKAI_TRUE if there is a fit, false in case no circles can be fit
//
bool Zikai::CircleFit(const std::vector<double>& x, const std::vector<double>& y, int N, const std::vector<cv::Point>& stEdMid, double pxc, double pyc, double pr, double pe, double angle)
{
	//*pe = 0;//precision
	if (N < 3) return false;

	double xAvg = 0;
	double yAvg = 0;

	for (int i = 0; i < N; i++) {
		xAvg += x[i];
		yAvg += y[i];
	} //end-for

	xAvg /= N;
	yAvg /= N;

	double Suu = 0;
	double Suv = 0;
	double Svv = 0;
	double Suuu = 0;
	double Suvv = 0;
	double Svvv = 0;
	double Svuu = 0;
	for (int i = 0; i < N; i++) {
		double u = x[i] - xAvg;
		double v = y[i] - yAvg;

		Suu += u * u;
		Suv += u * v;
		Svv += v * v;
		Suuu += u * u * u;
		Suvv += u * v * v;
		Svvv += v * v * v;
		Svuu += v * u * u;
	} //end-for

	// Now, we solve for the following linear system of equations
	// Av = b, where v = (uc, vc) is the center of the circle
	//
	// |N    Suv| |uc| = |b1|
	// |Suv  Svv| |vc| = |b2|
	//
	// where b1 = 0.5*(Suuu+Suvv) and b2 = 0.5*(Svvv+Svuu)
	//
	double detA = Suu * Svv - Suv * Suv;
	if (detA == 0) return false;

	double b1 = 0.5 * (Suuu + Suvv);
	double b2 = 0.5 * (Svvv + Svuu);

	double uc = (Svv * b1 - Suv * b2) / detA;
	double vc = (Suu * b2 - Suv * b1) / detA;

	double R = sqrt(uc * uc + vc * vc + (Suu + Svv) / N);

	pxc = uc + xAvg;
	pyc = vc + yAvg;


	pr = R;
	pe = 0;//*pe is the ZIKAI_TRUE MSE
	angle = 0;




	/*------------compute inliers after fitting------------*/
	/* first compute the distance from the edge point to the fitted circle, the distance constraint is
	then compute the angle between point normals and circle normals,
	if these two constraints hold, further compute the inlier ratio and the span angle.
	*/

	//first compute circular points' normals using the contour method
	std::vector<cv::Point2f> DT;
	std::vector<cv::Point2f> D1(N), D2(N);
	for (int j = 0; j < N - 1; j++)
	{   // differences between two consecutive points
		float L = sqrt(pow(x[j] - x[j + 1], 2) + pow(y[j] - y[j + 1], 2));
		D1[j].x = (x[j] - x[j + 1]) / L;
		D1[j].y = (y[j] - y[j + 1]) / L;
		D2[j + 1].x = (x[j] - x[j + 1]) / L;
		D2[j + 1].y = (y[j] - y[j + 1]) / L;
	}//endfor
	float LL = sqrt(pow(x.back() - x.front(), 2) + pow(y.back() - y.front(), 2));
	cv::Point2f end2start;
	end2start.x = (x.back() - x.front()) / LL;
	end2start.y = (y.back() - y.front()) / LL;
	D1.push_back(end2start);
	D2.front() = end2start;
	//D=D1+D2 Normal
	std::vector<cv::Point2f> D(N), Nom(N);
	for (int i = 0; i < N; i++)
	{
		D[i].x = D1[i].x + D2[i].x;
		D[i].y = D1[i].y + D2[i].y;
		double LL1 = sqrt(pow(D[i].x, 2) + pow(D[i].y, 2));
		Nom[i].x = -D[i].y / LL1;
		Nom[i].y = D[i].x / LL1;
	}//endfor


	//compute circular points' normals using fitted centers(equation)
	int inlierNum = 0;
	for (int i = 0; i < N; i++) {
		double dx = x[i] - pxc;

		double dy = y[i] - pyc;
		//point normals connecting the center
		double distP2O = sqrt(pow(dx, 2) + pow(dy, 2));
		double d = abs(distP2O - R);
		if (d <= 2)// distance less than 2 pixel
		{
			float theta = atan2(dy, dx);
			cv::Point2f circleNormal = cv::Point2f(cos(theta), sin(theta));
			float cosPtCirNormal2 = (Nom[i].x * circleNormal.x + Nom[i].y * circleNormal.y);
			if (abs(cosPtCirNormal2) >= cos(20.0 / 180 * CV_PI))// angle<=22.5°remember float calculations
			{
				inlierNum++;
			}//endif

		}//endif
		//waitKey();
	} //end-for

	double inlierEdgeRatio = (double)inlierNum / N;
	double inlierRatio = 0, spanAngle = 0;

	if (inlierEdgeRatio >= 0.5)// take up 0.8 of edge points
	{
		cv::Point center = cv::Point(pxc, pyc);

		inlierRatio = inlierNum / (2 * CV_PI * R);
		// compute span angles by the three points
		int num = stEdMid.size() / 3;
		for (int k = 0; k < num; k++)
		{
			cv::Point stPt(stEdMid[3 * k]);
			cv::Point edPt(stEdMid[3 * k + 1]);
			cv::Point midPt(stEdMid[3 * k + 2]);

			cv::Point o2st = cv::Point(stPt.x - pxc, stPt.y - pyc);
			cv::Point o2ed = cv::Point(edPt.x - pxc, edPt.y - pyc);
			double o2stL = sqrt(pow(o2st.x, 2) + pow(o2st.y, 2));
			double o2edL = sqrt(pow(o2ed.x, 2) + pow(o2ed.y, 2));
			double cosSA = (o2st.x * o2ed.x + o2st.y * o2ed.y) / (o2stL * o2edL);
			// calculate the line equation
			double Kse = (edPt.y - stPt.y) / (edPt.x - stPt.x + 1e-6);// in case of denominator=0
			// the sign of MidPt
			double signMid = ((Kse * (midPt.x - stPt.x) + stPt.y - midPt.y) > 0 ? 1 : -1);
			double signCenter = ((Kse * (center.x - stPt.x) + stPt.y - center.y) > 0 ? 1 : -1);

			// span angle
			double angle = acos((cosSA > 0.99) ? 0.99 : ((cosSA < -0.99) ? -0.99 : cosSA));
			if (signMid * signCenter > 0) { spanAngle += (2 * CV_PI - angle); }// superior arc
			else { spanAngle += angle; }// inferior arc
		}//endfor
	}//endif

	pr = R;
	pe = inlierRatio;
	angle = spanAngle;
	//}//endif

	return true;
}






/*----------circle verification using estimated centers and radii-------------------*/
bool Zikai::circleVerify(const std::vector<double>& x, const std::vector<double>& y, int N, const std::vector<cv::Point>& stEdMid, cv::Point2f O, double R, double& pe, double& angle)
{

	pe = 0;
	angle = 0;
	double pxc = O.x;
	double pyc = O.y;


	if (isinf(R) || isnan(R) || (R <= 0))
	{
		return false;
	}
	else
	{
		/*------------compute inliers after fitting------------*/
		/* first compute the distance from the edge point to the fitted circle, the distance constraint is
		then compute the angle between point normals and circle normals,
		if these two constraints hold, further compute the inlier ratio and the span angle.
		*/

		//first compute circular points' normals using the contour method
		std::vector<cv::Point2f> DT;
		std::vector<cv::Point2f> D1(N), D2(N);
		for (int j = 0; j < N - 1; j++)
		{   // differences between two consecutive points
			float L = sqrt(pow(x[j] - x[j + 1], 2) + pow(y[j] - y[j + 1], 2));
			D1[j].x = (x[j] - x[j + 1]) / L;
			D1[j].y = (y[j] - y[j + 1]) / L;
			D2[j + 1].x = (x[j] - x[j + 1]) / L;
			D2[j + 1].y = (y[j] - y[j + 1]) / L;
		}//endfor
		float LL = sqrt(pow(x.back() - x.front(), 2) + pow(y.back() - y.front(), 2));
		cv::Point2f end2start;
		end2start.x = (x.back() - x.front()) / LL;
		end2start.y = (y.back() - y.front()) / LL;
		D1.push_back(end2start);
		D2.front() = end2start;
		//D=D1+D2 Normal
		std::vector<cv::Point2f> D(N), Nom(N);
		for (int i = 0; i < N; i++)
		{
			D[i].x = D1[i].x + D2[i].x;
			D[i].y = D1[i].y + D2[i].y;
			double LL1 = sqrt(pow(D[i].x, 2) + pow(D[i].y, 2));
			Nom[i].x = -D[i].y / LL1;
			Nom[i].y = D[i].x / LL1;
		}//endfor


		//compute circular points' normals using fitted centers(equation)
		int inlierNum = 0;
		for (int i = 0; i < N; i++) {
			double dx = x[i] - pxc;

			double dy = y[i] - pyc;
			//point normals connecting the center
			double distP2O = sqrt(pow(dx, 2) + pow(dy, 2));
			double d = abs(distP2O - R);
			if (d <= 2)// distance less than 2 pixel
			{

				float theta = atan2(dy, dx);
				cv::Point2f circleNormal = cv::Point2f(cos(theta), sin(theta));

				float cosPtCirNormal2 = (Nom[i].x * circleNormal.x + Nom[i].y * circleNormal.y);
				if (abs(cosPtCirNormal2) >= cos(22.5 / 180 * CV_PI))// angle<=22.5°remember float calculations
				{
					inlierNum++;
				}//endif

			}//endif
			//waitKey();
		} //end-for

		double inlierEdgeRatio = (double)inlierNum / N;
		double inlierRatio = 0, spanAngle = 0;

		if (inlierEdgeRatio >= 0.2)// take up 0.5\0.8 of edge points
		{

			inlierRatio = inlierNum / (2 * CV_PI * R);

		}//endif
		pe = inlierRatio;
		angle = spanAngle;
	}//endelse
	return true;
}


std::vector<Zikai::Circle> Zikai::circleFitGroupedArcs(const std::vector<std::vector<cv::Point>>& groupedArcs, const std::vector<std::vector<cv::Point>>& groupedArcsThreePt)
{


	std::vector<Circle> addCircles;
	addCircles.reserve(groupedArcs.size());

	for (int i = 0; i < groupedArcs.size(); i++)
	{
		Circle fitCircle;
		std::vector<double> X, Y;// point coordinates
		std::vector<cv::Point> stEdMid;// arcs start, mid and end points;
		stEdMid = groupedArcsThreePt[i];

		for (int j = 0; j < groupedArcs[i].size(); j++)
		{	/* or for (auto j = groupedArcs[i].begin(); j != groupedArcs[i].end(); j++)*/
			Y.push_back(groupedArcs[i][j].y);// or X.push_back((*j).x)
			X.push_back(groupedArcs[i][j].x);
		}//endfor

		//fit
		double xc = -1.;
		double yc = -1.;
		double r = -1.;
		double inlierRatio = 0.;
		double spanAngle = 0.;
		CircleFit(X, Y, X.size(), stEdMid, xc, yc, r, inlierRatio, spanAngle);


		// inlier verification
		// You can tune inlierRation for better performance
		if (inlierRatio >= 0.3 && r >= 1 && spanAngle >= 1)
		{
			fitCircle.xc = xc;
			fitCircle.yc = yc;
			fitCircle.r = r;
			fitCircle.inlierRatio = inlierRatio;
			addCircles.push_back(fitCircle);
		}//endif

	}//endfor
	return addCircles;
}

/*------------fit circles using closed arcs---------*/
std::vector<Zikai::Circle> Zikai::circleFitClosedArcs(const std::vector<std::vector<cv::Point>>& closedArcs)
{


	std::vector<Circle> addCircles;
	addCircles.reserve(closedArcs.size());

	for (int i = 0; i < closedArcs.size(); i++)
	{
		Circle fitCircle;
		std::vector<double> X, Y;
		std::vector<cv::Point> threePt;

		for (int j = 0; j < closedArcs[i].size(); j++)
		{	/* or for (auto j = groupedArcs[i].begin(); j != groupedArcs[i].end(); j++)*/
			Y.push_back(closedArcs[i][j].y);// or X.push_back((*j).x)
			X.push_back(closedArcs[i][j].x);
		}//endfor

		//fit
		//double xc, yc, r, inlierRatio, spanAngle;
		double xc = -1.;
		double yc = -1.;
		double r = -1.;
		double inlierRatio = 0.;
		double spanAngle = 0.;
		CircleFit(X, Y, X.size(), threePt, xc, yc, r, inlierRatio, spanAngle);


		// inlier verification
		if (inlierRatio >= 0.5 && r >= 1)//&& closed arcs needn't compute spanAngles
		{
			fitCircle.xc = xc;
			fitCircle.yc = yc;
			fitCircle.r = r;
			fitCircle.inlierRatio = inlierRatio;
			addCircles.push_back(fitCircle);

		}//endif

	}//endfor
	return addCircles;
}



// cluster circles

bool cmpInlier(const Zikai::Circle& a, const Zikai::Circle& b)
{
	return a.inlierRatio > b.inlierRatio;
}//endbool


std::vector<Zikai::Circle> Zikai::clusterCircles(std::vector<Circle> totalCircles)
{
	std::vector<Circle> repCircles;// representative circles

	while (!totalCircles.empty()) {
		std::vector<Circle> simCircles;// similar circles

		Circle circle1 = totalCircles.front();

		for (auto it = totalCircles.begin(); it != totalCircles.end();)
		{

			Circle circle2 = *it;
			double disCircles = sqrt(pow(circle1.xc - circle2.xc, 2) + pow(circle1.yc - circle2.yc, 2) + pow(circle1.r - circle2.r, 2));
			if ((disCircles <= 5))// cluster 5\10
			{

				simCircles.push_back(circle2);// put together

				it = totalCircles.erase(it);// delete it
			}
			else { it++; }

		}
		// find the maximum inlier ratio in simCircles
		if (simCircles.size() > 1)
		{
			sort(simCircles.begin(), simCircles.end(), cmpInlier);
		}// sort default: increase order; cmpInlier is decrease order
		repCircles.push_back(simCircles.front());
		//std::cout << "Final inlier ratio" << simCircles.front().inlierRatio << std::endl;

	}
	return repCircles;
}




/*----------verify the circles by inlier ratio----------*/
std::vector<Zikai::Circle> Zikai::circleEstimateGroupedArcs(
	const std::vector<std::vector<cv::Point>>& groupedArcs,
	const std::vector<cv::Vec3f>& recordOR,
	const std::vector<std::vector<cv::Point>>& groupedArcsThreePt,
	float T_inlier,
	float T_angle
)
{


	std::vector<Circle> addCircles;
	addCircles.reserve(groupedArcs.size());

	for (int i = 0; i < groupedArcs.size(); i++)
	{
		Circle fitCircle;
		std::vector<double> X, Y;// point coordinates
		std::vector<cv::Point> stEdMid;// arcs start, mid and end points;
		stEdMid = groupedArcsThreePt[i];
		double groupedR = recordOR[i][2];
		cv::Point2f groupedO(recordOR[i][0], recordOR[i][1]);
		for (int j = 0; j < groupedArcs[i].size(); j++)
		{	/* or for (auto j = groupedArcs[i].begin(); j != groupedArcs[i].end(); j++)*/
			Y.push_back(groupedArcs[i][j].y);// or X.push_back((*j).x)
			X.push_back(groupedArcs[i][j].x);

		}//endfor

		//fit
		double  inlierRatio, spanAngle;
		circleVerify(X, Y, X.size(), stEdMid, groupedO, groupedR, inlierRatio, spanAngle);


		// inlier verification
		if (inlierRatio >= T_inlier && groupedR >= 5)//&& spanAngle>=1 && spanAngle<=0 && spanAngle >= T_angle spanned angle can be omitted
		{
			fitCircle.xc = groupedO.x;
			fitCircle.yc = groupedO.y;
			fitCircle.r = groupedR;
			fitCircle.inlierRatio = inlierRatio;

			addCircles.push_back(fitCircle);

		}//endif

	}//endfor                                                                      
	return addCircles;
}



/*------------estimate circles using closed arcs---------*/
std::vector<Zikai::Circle> Zikai::circleEstimateClosedArcs(std::vector<std::vector<cv::Point>> closedArcs, float T_inlier_closed)
{


	cv::Mat pre = cv::Mat(500, 700, CV_8UC3, cv::Scalar(255, 255, 255));


	std::vector<Circle> addCircles;

	for (int i = 0; i < closedArcs.size(); i++)
	{

		Circle fitCircle;
		std::vector<double> X, Y;
		std::vector<cv::Point> threePt;
		double closedR;
		cv::Point2f closedO;
		estimateClosedCenterRadius(closedArcs[i], &closedR, &closedO);// estimate the center and radius
		//std::cout << i << "ClosedCenter: " << closedO.x << " " << closedO.y << std::endl;

		int r = rand() % 256;
		int g = rand() % 256;
		int b = rand() % 256;
		cv::Scalar colorClosedEdgesPre = cv::Scalar(b, g, r);
		for (int j = 0; j < closedArcs[i].size(); j++)
		{	/* or for (auto j = groupedArcs[i].begin(); j != groupedArcs[i].end(); j++)*/
			Y.push_back(closedArcs[i][j].y);// or X.push_back((*j).x)
			X.push_back(closedArcs[i][j].x);
			circle(pre, closedArcs[i][j], 1, colorClosedEdgesPre);
		}//endfor

		//fit
		double inlierRatio, spanAngle;
		circleVerify(X, Y, X.size(), threePt, closedO, closedR, inlierRatio, spanAngle);


		// inlier verification

		if (inlierRatio >= 0.5 && closedR >= 4)//inlierRatio >=0.5 
		{
			fitCircle.xc = closedO.x;
			fitCircle.yc = closedO.y;
			fitCircle.r = closedR;
			fitCircle.inlierRatio = inlierRatio;
			addCircles.push_back(fitCircle);

		}//endif
		//std::cout << "Add circle done" << std::endl;
	}//endfor
	return addCircles;
}


/*convert str to num*/
template <typename T>
std::string to_string(T value)

{

	std::ostringstream os;

	os << value;

	return os.str();

}





// draw fitted circles
cv::Mat Zikai::drawResult(bool onImage, cv::Mat srcImg, std::string srcImgName, const std::vector<Circle>& circles)
{
	cv::Mat colorImage;
	int height = srcImg.rows;
	int width = srcImg.cols;
	if (onImage)
	{

		colorImage = srcImg;

	}
	else
		colorImage = cv::Mat(height, width, CV_8UC3, cv::Scalar(255, 255, 255));




	for (int i = 0; i < circles.size(); i++)
	{

		cv::Point2f center;
		float r;
		if (circles[i].r < 1)//You can remove very small circles (usually formed by noise)
		{
			center = cv::Point2f(circles[i].xc, circles[i].yc);
			r = circles[i].r;
			continue;
			circle(colorImage, center, r, cv::Scalar(90, 174, 25), 2, cv::LINE_AA);
		}
		else {
			center = cv::Point2f(circles[i].xc, circles[i].yc);
			r = circles[i].r;
			circle(colorImage, center, r, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);

		}

	}

	cv::imwrite(srcImgName, colorImage);
	return colorImage;
}




/*-----------load ground truth txt files-------------*/

void Zikai::LoadGT(std::vector<Circle>& gt, const std::string& sGtFileName)
{
	std::ifstream in(sGtFileName);
	if (!in.good())
	{
		//std::cout << "Error opening: " << sGtFileName << std::endl;
		return;
	}

	unsigned n;
	in >> n;

	gt.clear();
	gt.reserve(n);

	while (in.good() && n--)
	{
		Circle c;

		in >> c.xc >> c.yc >> c.r;
		gt.push_back(c);
	}
	in.close();
}






/*----------compute precision, recall and F-measure------------*/
bool Zikai::TestOverlap(const cv::Mat1b& gt, const cv::Mat1b& test, float th)
{
	float fAND = float(countNonZero(gt & test));
	float fOR = float(countNonZero(gt | test));
	float fsim = fAND / fOR;
	return (fsim >= th);
}

int Zikai::Count(const std::vector<bool> v)
{
	int counter = 0;
	for (unsigned i = 0; i < v.size(); ++i)
	{
		if (v[i]) { ++counter; }
	}
	return counter;
}


Zikai::pre_rec_fmeasure Zikai::Evaluate(const std::vector<Circle>& ellGT, const std::vector<Circle>& ellTest, const float th_score, const cv::Mat& img)
{
	float threshold_overlap = th_score;
	//float threshold = 0.95f;

	unsigned sz_gt = ellGT.size();
	unsigned size_test = ellTest.size();

	unsigned sz_test = unsigned(std::min(1000, int(size_test)));

	std::vector<cv::Mat1b> gts(sz_gt);
	std::vector<cv::Mat1b> tests(sz_test);
	//绘制每个目标椭圆
	for (unsigned i = 0; i < sz_gt; ++i)
	{
		const Circle& e = ellGT[i];

		cv::Mat1b tmp(img.rows, img.cols, uchar(0));

		circle(tmp, cv::Point((int)e.xc, (int)e.yc), (int)e.r, cv::Scalar(255), -1);
		gts[i] = tmp;
	}
	//plot the detected circles
	for (unsigned i = 0; i < sz_test; ++i)
	{
		const Circle& e = ellTest[i];

		cv::Mat1b tmp(img.rows, img.cols, uchar(0));
		circle(tmp, cv::Point((int)e.xc, (int)e.yc), (int)e.r, cv::Scalar(255), -1);

		tests[i] = tmp;
	}

	cv::Mat1b overlap(sz_gt, sz_test, uchar(0));

	for (int r = 0; r < overlap.rows; ++r)
	{
		for (int c = 0; c < overlap.cols; ++c)
		{

			overlap(r, c) = TestOverlap(gts[r], tests[c], threshold_overlap) ? uchar(255) : uchar(0);
		}
	}

	int counter = 0;
	std::vector<bool> vec_gt(sz_gt, false);

	for (unsigned int i = 0; i < sz_test; ++i)
	{
		//const Ellipse& e = ellTest[i];
		for (unsigned int j = 0; j < sz_gt; ++j)
		{
			if (vec_gt[j]) { continue; }

			bool bTest = overlap(j, i) != 0;

			if (bTest)
			{
				vec_gt[j] = true;
				break;
			}
		}
	}

	int tp = Count(vec_gt);

	int fn = int(sz_gt) - tp;
	int fp = size_test - tp; // !!!!

	float pr(0.f);
	float re(0.f);
	float fmeasure(0.f);

	if (tp == 0)
	{
		if (fp == 0)
		{
			pr = 1.f;
			re = 0.f;
			fmeasure = (2.f * pr * re) / (pr + re);
		}
		else
		{
			pr = 0.f;
			re = 0.f;
			fmeasure = 0.f;
		}
	}
	else
	{
		pr = float(tp) / float(tp + fp);
		re = float(tp) / float(tp + fn);
		fmeasure = (2.f * pr * re) / (pr + re);
	}

	pre_rec_fmeasure totalResult;
	totalResult.precision = pr;
	totalResult.recall = re;
	totalResult.fmeasure = fmeasure;
	return totalResult;
}


