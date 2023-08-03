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
#include "image.h"
#include <cstdlib>
#include <time.h>
#include <cmath>
#include "Data_structure.h"
#include <chrono>

#define kernel 7
#define block  (float)49 //block 7x7
#define LRCthr 2
#define PI 3.1416
#define img_W 450
#define img_H 375

using namespace std;


struct C_vol {
	//int x, y;
	float vc[255];
};

void Coordinate_trans(Point3f p, Point2i &pd);

void image_matching_SAD(const Image img, const Image img2)
{
	Image gray(img.width, img.height, 1);// 1 is channel
	//int kernel = 7;
	int h_size = (kernel - 1) / 2;
	float error = 9999, SAD;

	for (int y = h_size; y < img.height - h_size; y++) {
		for (int x = h_size; x < img.width - h_size; x++) {
			error = 9999;

			for (int d = 0; d < 60; d++)
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

void image_matching_NCC(const Image img, const Image img2, Image &disp, C_vol** arr, string type)
{
	
	int h_size = (kernel - 1) / 2;
	int c, q;
	float result, std_R, std_L,nominator;
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
							arr[y][x].vc[i] = NCC_Coef;																				//printf("%.4f\n", NCC_Coef);
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


float *intergrate_img(const Image img, float *integral_img)//製作積分圖
{
	integral_img[0] = img.get_pixel(0, 0, 0);

	/// Generate integral image,ex is reference to picture for my ppt. 
	for (int r = 0; r < img.height; r++) {
		for (int c = 1; c < img.width; c++) {
			integral_img[r*img.width + c] = integral_img[r*img.width + (c - 1)] + img.get_pixel(c, r, 0);// ex: (20 +30=50)
		}
	}	
	for (int r = 1; r < img.height; r++){
		for (int c = 0; c < img.width; c++)	{
			integral_img[r*img.width + c] = integral_img[(r - 1)*img.width + c ] + img.get_pixel(c, r, 0);// ex: (20 +30=50)		
			//printf(" finally  %d %d = %.4f  \n", c, r, integral_img[r*img.width + c]);
		}		
	}

	return integral_img;
}

float GetValueID(float *integral_img, int x, int y, int n,int W)//從積分圖取得 N x N 區塊的總和 
{
	float value;
	float A, B, C, D;
	A = integral_img[(y + n)*W + (x + n)];//A
	B = (x - n - 1 < 0) ? 0 : integral_img[(y + n)*W + (x - n - 1)];
	C = (y - n - 1 < 0) ? 0 : integral_img[(y - n - 1)*W + (x + n)];//三元運算子 條件為false ,否則計算
	D = ((y - n - 1 < 0) || (x - n - 1 < 0)) ? 0 : integral_img[(y - n - 1)*W + (x - n - 1)];

	value = A - B - C + D;
	return value;
}

void block_mean_std(const Image img, float *std, float *mean)
{
	int bound = (kernel - 1) / 2;
	Image img_rel;
	//使用積分圖加速(事先計算好區塊)	
	int total = kernel * kernel;
	int W = img.width;
	float sum;
	float *integral_img = (float*)malloc(sizeof(float)*img.width*img.height);

	integral_img = intergrate_img(img, integral_img);

	// pos  座標儲存位置
	unsigned long pos;//( x,y )二維坐標軸轉成一維坐標軸標定

	for (int j = bound; j < img.height - bound; j++){
		for (int i = bound; i < img.width- bound; i++){
			pos = j*img.width + i;//位置
			sum = GetValueID(integral_img, i, j, bound, W);
			mean[pos] = sum / block;
			/*printf("(%d, %d) = %.4f \n", i, j, sum);*/
			std[pos] = 0;
			for (int a = -bound; a <= bound; a++){
				for (int b = -bound; b <= bound; b++){
					float diff = img.get_pixel(i + b, j + a, 0) - mean[pos];
					//printf("diff = %.2f\n", diff);
					std[pos] += diff*diff;//2次方
				}
			}
			std[pos] = sqrt(std[pos] / block);//standard diff
		}
	}

	free(integral_img);
}

//RU ZP HK4GG4 test
void Fast_NCC(const Image img_R, const Image img_L,Image &disp,string type)
{
	int size = img_R.height*img_R.width;
	int W = img_R.width;
	float *std_R = (float*)malloc(sizeof(float)*size);
	float *mean_R = (float*)malloc(sizeof(float)*size);
	float *std_L = (float*)malloc(sizeof(float)*size);
	float *mean_L = (float*)malloc(sizeof(float)*size);

	block_mean_std(img_R, std_R, mean_R);
	block_mean_std(img_L, std_L, mean_L);

	unsigned long pos, pos2;//( x,y )二維坐標軸轉成一維坐標軸標定
	int h_size = (kernel - 1) / 2;//block 一半
	int c, q, d = 60;//search_range
	float result, nominator, NCC_Coef;
	float e[kernel*kernel], a[kernel*kernel];
	float disp_f;


	if (type == "left") {//left image and right image implement image matching
		for (int y = h_size; y < img_L.height - h_size; y++) {
			for (int x = img_L.width - h_size; x >= h_size; x--) {
				c = 0;
				for (int t = -h_size; t <= h_size; t++) {
					for (int r = -h_size; r <= h_size; r++) {
						e[c] = img_L.get_pixel(x + r, y + t, 0);
						c++;
					}
				}
				result = 0;
				pos = y*W + x ;//位置
				for (int j = y; j <= y; j++){
					for (int i = 0; i < d; i++){
						if (j - h_size > 0 && (x - i) - h_size > 0) {
							NCC_Coef = 0;
							q = 0;
							nominator = 0;
							pos2 = j*W + (x - i);//位置
							// range N x N block
							for (int rr = j - h_size; rr <= (j + h_size); rr++){
								for (int tt = (x - i) - h_size; tt <= (x - i) + h_size; tt++){
									a[q] = img_R.get_pixel(tt, rr, 0);
									q++;
								}
							}

							for (int l = 0; l < q; l++) {
								nominator += (e[l] - mean_L[pos])*(a[l] - mean_R[pos2]);
							}

							NCC_Coef = fabs(nominator / block) / (std_R[pos2] * std_L[pos]); //NCC 計算
							//printf("%.4f\n", NCC_Coef);
							if (NCC_Coef > result){
								result = NCC_Coef;
								disp_f = i;
								disp.set_pixel(x, y, 0, disp_f / 255.0);//disparity map	  //gray is set normalize (0~1) 相當於 (0-255) 
							}
						}
					}
				}
			}
		}
	}
	else {
		for (int y = h_size; y < img_L.height - h_size; y++) {
			for (int x = h_size; x < img_L.width - h_size; x++) {
				c = 0;
				for (int t = -h_size; t <= h_size; t++) {
					for (int r = -h_size; r <= h_size; r++) {
						e[c] = img_R.get_pixel(x + r, y + t, 0);
						c++;
					}
				}
				pos = y*W + x;//位置
				result = 0;
				for (int j = y; j <= y; j++){
					for (int i = 0; i < d; i++){
						if (i + h_size < img_L.width) {
							NCC_Coef = 0;
							q = 0;
							nominator = 0;
							pos2 = j*W + (x + i);//位置
							// range N x N block
							for (int rr = j - h_size; rr <= (j + h_size); rr++){
								for (int tt = x + i - h_size; tt <= (x + i + h_size); tt++){
									a[q] = img_L.get_pixel(tt, rr, 0);
									q++;
								}
							}

							for (int l = 0; l < q; l++) {
								nominator += (a[l] - mean_L[pos2])*(e[l] - mean_R[pos]);}

							NCC_Coef = fabs(nominator / block) / (std_L[pos2] * std_R[pos]); //NCC 歸一化計算
						
							if (NCC_Coef > result){
								result = NCC_Coef;
								disp_f = i;
								disp.set_pixel(x, y, 0, disp_f / 255.0);//disparity map	//gray is set normalize (0~1) 相當於 (0-255) 												   
							}
						}
					}
				}
			}
		}
	}

	free(mean_R);
	free(mean_L);
	free(std_R);
	free(std_L);
}

void Displacement_Checky(Image dshift_R, Image dshift_L, Image &Chk_img)
{
	float d, df;
	float th = LRCthr;
	th /= 255;
	for (int j = 0; j < dshift_R.height; j++){
		for (int i = 0; i < dshift_R.width; i++){
			df = dshift_R.get_pixel(i, j, 0);
			d = df*255.0;
			if (abs(dshift_L.get_pixel(i + d, j, 0) - df) <= th)
				Chk_img.set_pixel(i, j, 0, df);
		}
	}
}


void SubPixel(Image img, C_vol **dsi, Image &sub)
{
	for (int j = 0; j < img.height; j++)
	{
		for (int i = 0; i < img.width; i++)
		{
			int d = img.get_pixel(i,j,0)*255;
			//printf("disp = %d \n", d);
			if (d == 0||d>255)
				continue;
			
			//影像比對的信賴度
			float cd1 = dsi[j][i].vc[d - 1];
			float cd2 = dsi[j][i].vc[d];
			float cd3 = dsi[j][i].vc[d + 1];

			float ds = d + (cd1 - cd3) / (2 * cd1 + 2 * cd3 - 4 * cd2);

			//printf("sub disp = %.2f \n", ds);
			if (ds > 0 && ds <= 255)
				sub.set_pixel(i, j, 0, ds/ 255); //(x, y, 0, dr );
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
	Image sub(img.width, img.height, 1);

	int r = img.height, c = img.width, i, j;

	C_vol** arr = (C_vol**)malloc(r * sizeof(C_vol*));
	for (i = 0; i < r; i++)
		arr[i] = (C_vol*)malloc(c * sizeof(C_vol));
	
	auto start = chrono::steady_clock::now();
	//
	//	// 在這裡做一些事情

	image_matching_NCC(img, img2, disp, arr, "right");
	image_matching_NCC(img, img2, disp2, arr, "left");


	auto end = chrono::steady_clock::now();
	
	cout << "Elapsed time in seconds: "
		<< chrono::duration_cast<chrono::seconds>(end - start).count()
		<< " sec";
	disp.save("disp.jpg");
	disp2.save("disp2.jpg");
	Image Chk_img(img.width, img.height, 1);// 1 is channel

	Displacement_Checky(disp, disp2, Chk_img);
	Chk_img.save("Chk_img.jpg");
	//

	
	SubPixel(Chk_img, arr, sub);

	float f = 4.0;
	float sb = 3.2*pow(10, -3); 
	float baseline=40, Z;
	float disp_f;
	Point2i pd;//3D
	Point3f img_p;//資料結構: 輸入影像座標與視差值
	//float r_temp, g_temp, b_temp;
	uint8_t r_temp, g_temp, b_temp;
	string nn = "street_0510_map_full_.txt";//輸出存取
	FILE* Write_f = fopen(nn.c_str(), "w");
	int X_i, Y_i ;
	for (int j = 0; j < img.height; j++){
		for (int i = 0; i < img.width; i++)	{
			
			if (sub.get_pixel(i, j, 0) < 0.000001)
				continue;
			disp_f = sub.get_pixel(i, j, 0) * 255;
			Z = ((baseline*f) / (disp_f*sb));// //3d depth
			
			//printf("%.4f \n", Z);
			img_p.x = i; img_p.y = j; img_p.z = Z;
			Coordinate_trans(img_p, pd);

			r_temp = color_1.get_pixel(i, j, 0) * 255;
			g_temp = color_1.get_pixel(i, j, 1) * 255;
			b_temp = color_1.get_pixel(i, j, 2) * 255;
			
			X_i = pd.x;
			Y_i = pd.y;

			if (Z>10000)
				continue;
			fprintf(Write_f, "%d %d %.2f %d %d %d\n", X_i, Y_i, Z, r_temp, g_temp, b_temp);
		}
	}
	
	fclose(Write_f);

	for (int i = 0; i < r; i++)
		free(arr[i]);
	free(arr);

	system("pause");
	return 0;
}


//  Fast NCC  test  0509 have bug in right disparity map
//int main()
//{
//	Image color_1("D://im2.png");
//	Image color_2("D://im1.png");
//
//	Image img = rgb_to_grayscale(color_1);
//	Image img2 = rgb_to_grayscale(color_2);
//
//	//image_matching_SAD(img, img2);
//	Image disp(img.width, img.height, 1);// 1 is channel
//	Image disp2(img.width, img.height, 1);// 1 is channel
//	
//	auto start = chrono::steady_clock::now();
//
//	// 在這裡做一些事情
//	Fast_NCC(img, img2, disp, "right");
//	Fast_NCC(img, img2, disp2, "left");
//
//	auto end = chrono::steady_clock::now();
//
//	cout << "Elapsed time in seconds: "
//		<< chrono::duration_cast<chrono::seconds>(end - start).count()
//		<< " sec";
//
//	disp.save("disp.jpg");
//	disp2.save("disp2.jpg");
//
//	system("pause");
//	return 0;
//}

