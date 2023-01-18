#include <stdio.h>
#include <stdlib.h>
#include <math.h>//

unsigned char bmp_head[14]; //byte0-13 �Y��T 
unsigned char *bmp_information;//Ū���ɮסA�ϥΪŶ��j�p 

void four_byte_hex_to_ten(unsigned char *, int *, int, int);

int main(void)
{
	FILE *fptr1, *fptr2;
	int i, j;
	int imagestarbyte; //�_�l�줸�� ���(byte)  
	int bmp_information_space; //�ϥΪŶ��j�p
	int width, height;
	int pixel; //bmp size = width*height

	if ((fptr1 = fopen("D://gray_lena.bmp", "rb")) == NULL)
	{
		printf("�L�k�}��.\n");
		system("pause");
		exit(1);
	}
	if ((fptr2 = fopen("D://boats_gray.bmp", "wb")) == NULL)
	{
		printf("�L�k�إ�.\n");
		system("pause");
		exit(1);
	}
	fread(bmp_head, sizeof(unsigned char), 14, fptr1); //�qbmp�ɮ�Ū��14bytes, ��Jhead
													   // �N��Ʀs�J fptr2 14bytes 
	if (!(bmp_head[0] == 'B'&&bmp_head[1] == 'M'))
	{
		printf("not bmp image.\n");
		system("pause");
		exit(0);
	}
	fwrite(bmp_head, sizeof(unsigned char), 14, fptr2);//�Nbmp ���Y��T�s�Jfptr2,�@14bytes	
	four_byte_hex_to_ten(bmp_head, &imagestarbyte, 10, 13); //bytes 10-13
	bmp_information_space = imagestarbyte - 14;//�ϥΪŶ��j�p

											   //�ʺA�t�m���ɸ�T�� 
	bmp_information = (unsigned char*)malloc(sizeof(unsigned char)*bmp_information_space);
	//�A�qbmp��Ū�� bytes
	//�s�J�r���}�C 
	fread(bmp_information, sizeof(unsigned char), bmp_information_space, fptr1);

	//information_space �@�� byte
	fwrite(bmp_information, sizeof(unsigned char), bmp_information_space, fptr2);
	//////////////////////////////////////////////
	four_byte_hex_to_ten(bmp_information, &width, 4, 7); //bytes 
	four_byte_hex_to_ten(bmp_information, &height, 8, 11); //bytes 

	unsigned char bmppixel[512][512]; //8�줸 
	unsigned char result[512][512];   //8�줸 

	pixel = width*height;//���o�v���j�p 

	fread(bmppixel, sizeof(unsigned char), pixel, fptr1);//���obmp�ɮװ}�C��� 

	int H[256] = { 0 };//�s���Ƕ���0-255�X�{���v 

					   //�t�m�G���}�C�O����j�p 
	int **image_Y = (int**)malloc(width * sizeof(int**));
	for (i = 0; i<height; i++)
	{
		image_Y[i] = (int*)malloc(height * sizeof(int*));
	}

	printf("%d\n", height);
	for (i = 0; i<height; i++)
	{
		for (j = 0; j<width; j++)
		{
			image_Y[i][j] = bmppixel[i][j];//bmmpixel[i*width+j];
										   //printf("%d ",image_Y[i][j]);
			H[image_Y[i][j]]++;
		}
	}
	for (i = 0; i<255; i++)
		printf("gray level [%d]=%d\n", i, H[i]);
	float p[256] = { 0 };//�o�;��v
	float cdf[256] = { 0 };
	int cdf_min = 1; //cdf mininum
	int cdf_max = 1;

	for (i = 24; i < 256; i++)
	{
		p[i] = H[i];//�C�ӦǶ��Ȫ����v �N�O���� 
		cdf[i] = p[i] + cdf[i - 1]; //�ֿn 
									//�p�ⷥ�j�ȩM���p�� 
		if (cdf[i] < cdf_min && cdf[i] != 0)
			cdf_min = cdf[i];
		if (cdf[i] > cdf_max && cdf[i] != 0)
			cdf_max = cdf[i];
	}

	float h[256];
	for (i = 0; i<256; i++) {
		h[i] = round(((cdf[i] - cdf_min) / (cdf_max - cdf_min)) * 255);
	}
	int o[256];	//�d�ݷs���Ƕ�����
	for (i = 0; i < 256; i++) {
		o[i] = (int)h[i];
		//printf("new gray level [%d]=%d\n",i,o[i]);
	}
	for (i = 0; i < height; i++) {
		for (j = 0; j < width; j++) {
			result[i][j] = (int)h[image_Y[i][j]];
		}
	}
	fwrite(result, sizeof(char), pixel, fptr2);

	for (i = 0; i<height; i++)//�^���G�����O���� 
		free(image_Y[i]);
	free(image_Y);

	printf("ok finish.\n");
	fclose(fptr1);
	fclose(fptr2);

	system("pause");
	return 0;
}

//�C�줸�ը찪�줸�� 16-10�i��  
void four_byte_hex_to_ten(unsigned char *section, int *size, int beginbyte, int endbyte)
{
	int i, j;
	int lowbit;  //�C��bytes �� 0~3bit
	int highbit; //�C��bytes �� 4~7bit

	j = 1;
	*size = 0;
	for (i = beginbyte; i <= endbyte; i++)
	{
		lowbit = section[i] % 16;
		highbit = section[i] / 16;
		*size = *size + (int)pow(16, 2 * j - 1)*highbit + (int)pow(16, 2 * j - 2)*lowbit;
		j++;
	}
}
