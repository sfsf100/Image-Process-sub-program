///*
//* main.cpp
//*
//* Copyright (C) Lin Shu You, 2023
//*
//* This applies stereo vision Image matching.
//* Include SAD 、 NCC matcing, integral image, Displacement_Checky,sub disparity 
//* 
//* Last update date: 2023 /05 /10
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
#define LRCthr 2
#define img_W 427
#define img_H 370

using namespace std;

void Median_filter(Image img, Image &median);


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


void Displacement_Checky(Image dshift_R, Image dshift_L, Image &Chk_img)
{
	float d, df;
	float th = LRCthr;
	th /= 255;
	for (int j = 0; j < dshift_R.height; j++) {
		for (int i = 0; i < dshift_R.width; i++) {
			df = dshift_R.get_pixel(i, j, 0);
			d = df*255.0;
			if (abs(dshift_L.get_pixel(i + d, j, 0) - df) <= th)
				Chk_img.set_pixel(i, j, 0, df);
		}
	}
}

int main()
{
	Image color_1("D://view5.png");
	Image color_2("D://view1.png");

	Image img = rgb_to_grayscale(color_1);
	Image img2 = rgb_to_grayscale(color_2);

	Image disp(img.width, img.height, 1);// 1 is channel
	Image disp2(img.width, img.height, 1);// 1 is channel

	int r = img.height, c = img.width, i, j;


	auto start = chrono::steady_clock::now();

	image_matching_NCC(img, img2, disp, "right");
	image_matching_NCC(img, img2, disp2, "left");


	auto end = chrono::steady_clock::now();

	cout << "Elapsed time in seconds: "
		<< chrono::duration_cast<chrono::seconds>(end - start).count()
		<< " sec";
	disp.save("disp.jpg");
	disp2.save("disp2.jpg");
	Image Chk_img(img.width, img.height, 1);// 1 is channel

	Displacement_Checky(disp, disp2, Chk_img);
	Chk_img.save("Chk_img.jpg");
	////
	Image median_img(img.width, img.height, 1);// 1 is channel
	Median_filter(Chk_img, median_img);
	median_img.save("median_img.jpg");

	system("pause");
	return 0;
}
