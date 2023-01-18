#include <stdio.h>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <time.h>
#include <cmath>
#include <direct.h>
/*
* main.cpp
*
* Copyright (C) Lin Shu You, 2022
*
* This applies visual 3D edge compensation processing.
* Last update date: 2022 / 12 / 07
*
*/

#define BIN_WIDTH 0.5            
#define NUM_BINS 180 / BIN_WIDTH  
typedef unsigned short u_short;
using namespace std; //C++ »yªk¥Î
using namespace cv;

void inverse(Mat edge, Mat &result)
{
	result = Mat::zeros(edge.rows, edge.cols, CV_8U);
	for (int i = 0; i < edge.rows; ++i) {
		for (int j = 0; j < edge.cols; ++j) {
			if (edge.at<uchar>(i, j) == 255)
				result.at<uchar>(i, j) = 0;
			else
				result.at<uchar>(i, j) = 255;
		}
	}
}

void polarToCartesian(float rho, float theta, Point& p1, Point& p2)
{
	float a = cos((theta / 20.0)*(3.1416 / 180.0)), b = sin((theta / 20.0)*(3.1416 / 180.0));
	float x0 = a*rho, y0 = b*rho;

	p1.x = cvRound(x0 + 3000.0 * (-b));
	p1.y = cvRound(y0 + 3000.0 * (a));
	p2.x = cvRound(x0 - 3000.0 * (-b));
	p2.y = cvRound(y0 - 3000.0 * (a));
}


int main()
{
	Mat source = imread("D://0118.png");
	Mat gray, blur, dst;
	Point p1, p2;
	float theta, rho, rho_a, theta_a;
	int i_theta, i_rho;
	short max_x, max_y;
	int maxDistance, length;

	maxDistance = hypot(source.rows, source.cols);
	length = 2 * maxDistance;
	// accumulator volume by memory allocate 
	unsigned short **accumulator = (unsigned short **)calloc((length), sizeof(unsigned short*));
	for (int r = 0; r < (length); r++)
		accumulator[r] = (unsigned short *)calloc(3601, sizeof(unsigned short));

	Mat rho_theta_map = Mat::zeros(2 * maxDistance, 3600, CV_16U);
	cvtColor(source, gray, COLOR_BGR2GRAY);
	GaussianBlur(gray, blur, Size(5, 5), 1.5);
	Canny(blur, dst, 30, 40, 3);
	gray.release();//free Mat
	blur.release();//free Mat

	for (int i = 10; i < dst.rows - 10; ++i) {
		for (int j = 0; j < dst.cols; ++j) {
			if (dst.at<uchar>(i, j) == 255) {
				for (theta = 0; theta <= 180.0; theta += 0.05) {
					rho = round(j * cos((theta)*(3.1416 / 180.0)) + i * sin((theta)*(3.1416 / 180.0))) + maxDistance;//+ maxDistance shift positive value
					i_rho = rho;
					i_theta = theta * 20;
					accumulator[i_rho][i_theta]++;
					/*rho_theta_map.at<ushort>(i_rho, i_theta) = accumulator[i_rho][i_theta];*/
				}
			}
		}
	}
	Mat result_img, cdst;
	inverse(dst, result_img);

	cvtColor(result_img, cdst, COLOR_GRAY2BGR);
	int length_note = 0;

	for (int i = 0; i < length; ++i) { //rho 
		for (int j = 0; j < 3600; ++j) { //theta
			length_note = 0;
			if (accumulator[i][j] >= 200) {//printf("next Hough line  \n");			
				max_x = j;
				max_y = i;

				rho_a = max_y - maxDistance;
				theta_a = max_x;
				polarToCartesian(rho_a, theta_a, p1, p2);
				line(cdst, Point(p1.x, p1.y), Point(p2.x, p2.y), Scalar(255, 0, 0),
					2, LINE_8);


			}
		}
	}


	for (int r = 0; r < length; r++)
		free(accumulator[r]);//free 2d array memory
	free(accumulator);//free 1d array memory
	imshow("color", cdst);
	waitKey(0);
}
