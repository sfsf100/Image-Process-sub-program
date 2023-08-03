///*
//* main.cpp
//*
//* Copyright (C) Lin Shu You, 2023
//*
//* This applies stereo vision Image matching.
//* SAD 
//*
//*/
#include <iostream> 
#include <string>
#include<direct.h>
#include <cstdlib>
#include <time.h>
#include <cmath>
#include <chrono>
#include "image.h"

#define kernel 7
#define block  (float)49 //block 7x7
#define PI 3.1416
#define img_W 450
#define img_H 375
#define SR 60
using namespace std;


void image_matching_SAD(const Image img, const Image img2)
{
	Image gray(img.width, img.height, 1);// 1 is channel
										 //int kernel = 7;
	int h_size = (kernel - 1) / 2;
	float error = 9999, SAD;

	for (int y = h_size; y < img.height - h_size; y++) {
		for (int x = h_size; x < img.width - h_size; x++) {
			error = 9999;

			for (int d = 0; d < SR; d++)
			{
				SAD = 0;
				for (int t = -h_size; t <= h_size; t++) {
					for (int r = -h_size; r <= h_size; r++) {
						SAD += fabs(img.get_pixel(x + r, y + t, 0) - img2.get_pixel(x + r + d, y + t, 0));
					}
				}
				if (SAD < error) {
					error = SAD;
					float dr = d;
					gray.set_pixel(x, y, 0, dr / 255);
					//img.set_pixel(x, y, 0, 0.f);
				}
			}
		}
	}
	gray.save("Disp_img.png");
}

int main()
{
	Image color_1("D://im2.png");//Right image
	Image color_2("D://im1.png");//Left image

	Image img = rgb_to_grayscale(color_1);
	Image img2 = rgb_to_grayscale(color_2);

	image_matching_SAD(img, img2);

	system("pause");
	return 0;
}


///*
//* main.cpp
//*
//* Copyright (C) Lin Shu You, 2023
//*
//* This applies stereo vision Image matching.
//* Include NCC matcing
//* 
//*
//*/
#include <iostream> 
#include <string>
#include <cstdlib>
#include <time.h>
#include <cmath>
#include <chrono>
#include "image.h"

#define kernel 7
#define block  (float)49 //block 7x7
#define img_W 450
#define img_H 375

using namespace std;


void image_matching_NCC(const Image img, const Image img2, Image &disp, string type)
{

	int h_size = (kernel - 1) / 2;
	int c, q;
	float result, std_R, std_L, nominator;
	float NCC_Coef;
	float e[kernel*kernel], a[kernel*kernel];
	float mean_L, mean_R;
	int d = 60;//search_range
	if (type == "left") {//left image and right image implement image matching
		for (int y = h_size; y < img.height - h_size; y++) {
			for (int x = img.width - h_size; x >= h_size; x--) {
				mean_L = 0;
				c = 0;
				for (int t = -h_size; t <= h_size; t++) {
					for (int r = -h_size; r <= h_size; r++) {
						e[c] = img2.get_pixel(x + r, y + t, 0);
						mean_L += img2.get_pixel(x + r, y + t, 0);
						c++;
					}
				}
				mean_L /= (block);
				result = 0;

				for (int j = y; j <= y; j++)
				{
					for (int i = 0; i < d; i++)
					{
						if (j - h_size > 0 && (x - i) - h_size > 0) {
							NCC_Coef = 0;
							std_R = 0, std_L = 0;
							q = 0;
							nominator = 0;
							mean_R = 0;
							// range N x N block
							for (int rr = j - h_size; rr <= (j + h_size); rr++)
							{
								for (int tt = (x - i) - h_size; tt <= (x - i) + h_size; tt++)
								{
									a[q] = img.get_pixel(tt, rr, 0);
									mean_R += img.get_pixel(tt, rr, 0);
									q++;
								}
							}
							mean_R /= (block);

							for (int l = 0; l < q; l++) {
								nominator += (e[l] - mean_L)*(a[l] - mean_R);
								std_L += (e[l] - mean_L)*(e[l] - mean_L);
								std_R += (a[l] - mean_R)*(a[l] - mean_R);
							}

							NCC_Coef = fabs(nominator / block) / (sqrtf(std_L / block) * sqrtf(std_R / block)); //NCC 歸一化計算
																												//printf("%.4f\n", NCC_Coef);
							if (NCC_Coef > result)
							{
								result = NCC_Coef;
								float d = i;
								disp.set_pixel(x, y, 0, d / 255.0);//disparity map	
																   //gray is set normalize (0~1) 相當於 (0-255) 

							}
						}
					}
				}
			}
		}
	}
	else {
		for (int y = h_size; y < img.height - h_size; y++) {
			for (int x = h_size; x < img.width - h_size; x++) {
				mean_R = 0;
				c = 0;
				for (int t = -h_size; t <= h_size; t++) {
					for (int r = -h_size; r <= h_size; r++) {
						e[c] = img.get_pixel(x + r, y + t, 0);
						mean_R += img.get_pixel(x + r, y + t, 0);
						c++;
					}
				}
				mean_R /= (block);
				result = 0;
				for (int j = y; j <= y; j++)
				{
					for (int i = 0; i < d; i++)
					{
						if (i + h_size < img.width) {
							NCC_Coef = 0;
							std_R = 0, std_L = 0;
							q = 0;
							nominator = 0;
							mean_L = 0;
							// range N x N block
							for (int rr = j - h_size; rr <= (j + h_size); rr++)
							{
								for (int tt = x + i - h_size; tt <= (x + i + h_size); tt++)
								{
									a[q] = img2.get_pixel(tt, rr, 0);
									mean_L += img2.get_pixel(tt, rr, 0);
									q++;
								}
							}
							mean_L /= (block);

							for (int l = 0; l < q; l++) {
								nominator += (a[l] - mean_L)*(e[l] - mean_R);
								std_L += (a[l] - mean_L)*(a[l] - mean_L);
								std_R += (e[l] - mean_R)*(e[l] - mean_R);
							}

							NCC_Coef = fabs(nominator / block) / (sqrtf(std_L / block) * sqrtf(std_R / block)); //NCC 歸一化計算
																									//printf("%.4f\n", NCC_Coef);
							if (NCC_Coef > result)
							{
								result = NCC_Coef;
								float d = i;
								disp.set_pixel(x, y, 0, d / 255.0);//disparity map	
																   //gray is set normalize (0~1) 相當於 (0-255) 
							}
						}
					}
				}
			}
		}
	}
}


int main()
{
	Image color_1("D://im2.png");
	Image color_2("D://im1.png");

	Image img = rgb_to_grayscale(color_1);
	Image img2 = rgb_to_grayscale(color_2);

	//image_matching_SAD(img, img2);
	Image disp(img.width, img.height, 1);// 1 is channel
	Image disp2(img.width, img.height, 1);// 1 is channel

	int r = img.height, c = img.width, i, j;


	auto start = chrono::steady_clock::now();
	//
	//	// 在這裡做一些事情

	image_matching_NCC(img, img2, disp, "right");
	image_matching_NCC(img, img2, disp2,"left");


	auto end = chrono::steady_clock::now();

	cout << "Elapsed time in seconds: "
		<< chrono::duration_cast<chrono::seconds>(end - start).count()
		<< " sec";
	disp.save("disp.jpg");
	disp2.save("disp2.jpg");


	system("pause");
	return 0;
}