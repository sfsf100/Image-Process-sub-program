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
	gray.save("gray_img.png");
}

int main()
{
	Image color_1("D://im2.png");
	Image color_2("D://im1.png");

	Image img = rgb_to_grayscale(color_1);
	Image img2 = rgb_to_grayscale(color_2);
	
	image_matching_SAD(img, img2);
	
	system("pause");
	return 0;
}
