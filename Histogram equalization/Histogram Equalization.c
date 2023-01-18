#include <stdio.h>
#include <stdlib.h>
#include <math.h>//

unsigned char bmp_head[14]; //byte0-13 頭資訊 
unsigned char *bmp_information;//讀取檔案，使用空間大小 

void four_byte_hex_to_ten(unsigned char *, int *, int, int);

int main(void)
{
	FILE *fptr1, *fptr2;
	int i, j;
	int imagestarbyte; //起始位元組 單位(byte)  
	int bmp_information_space; //使用空間大小
	int width, height;
	int pixel; //bmp size = width*height

	if ((fptr1 = fopen("D://gray_lena.bmp", "rb")) == NULL)
	{
		printf("無法開啟.\n");
		system("pause");
		exit(1);
	}
	if ((fptr2 = fopen("D://boats_gray.bmp", "wb")) == NULL)
	{
		printf("無法建立.\n");
		system("pause");
		exit(1);
	}
	fread(bmp_head, sizeof(unsigned char), 14, fptr1); //從bmp檔案讀取14bytes, 放入head
													   // 將資料存入 fptr2 14bytes 
	if (!(bmp_head[0] == 'B'&&bmp_head[1] == 'M'))
	{
		printf("not bmp image.\n");
		system("pause");
		exit(0);
	}
	fwrite(bmp_head, sizeof(unsigned char), 14, fptr2);//將bmp 檔頭資訊存入fptr2,共14bytes	
	four_byte_hex_to_ten(bmp_head, &imagestarbyte, 10, 13); //bytes 10-13
	bmp_information_space = imagestarbyte - 14;//使用空間大小

											   //動態配置圖檔資訊區 
	bmp_information = (unsigned char*)malloc(sizeof(unsigned char)*bmp_information_space);
	//再從bmp黨讀取 bytes
	//存入字元陣列 
	fread(bmp_information, sizeof(unsigned char), bmp_information_space, fptr1);

	//information_space 共個 byte
	fwrite(bmp_information, sizeof(unsigned char), bmp_information_space, fptr2);
	//////////////////////////////////////////////
	four_byte_hex_to_ten(bmp_information, &width, 4, 7); //bytes 
	four_byte_hex_to_ten(bmp_information, &height, 8, 11); //bytes 

	unsigned char bmppixel[512][512]; //8位元 
	unsigned char result[512][512];   //8位元 

	pixel = width*height;//取得影像大小 

	fread(bmppixel, sizeof(unsigned char), pixel, fptr1);//取得bmp檔案陣列資料 

	int H[256] = { 0 };//存取灰階值0-255出現機率 

					   //配置二維陣列記憶體大小 
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
	float p[256] = { 0 };//發生機率
	float cdf[256] = { 0 };
	int cdf_min = 1; //cdf mininum
	int cdf_max = 1;

	for (i = 24; i < 256; i++)
	{
		p[i] = H[i];//每個灰階值的機率 就是次數 
		cdf[i] = p[i] + cdf[i - 1]; //累積 
									//計算極大值和極小值 
		if (cdf[i] < cdf_min && cdf[i] != 0)
			cdf_min = cdf[i];
		if (cdf[i] > cdf_max && cdf[i] != 0)
			cdf_max = cdf[i];
	}

	float h[256];
	for (i = 0; i<256; i++) {
		h[i] = round(((cdf[i] - cdf_min) / (cdf_max - cdf_min)) * 255);
	}
	int o[256];	//查看新的灰階分布
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

	for (i = 0; i<height; i++)//回收二維的記憶體 
		free(image_Y[i]);
	free(image_Y);

	printf("ok finish.\n");
	fclose(fptr1);
	fclose(fptr2);

	system("pause");
	return 0;
}

//低位元組到高位元組 16-10進制  
void four_byte_hex_to_ten(unsigned char *section, int *size, int beginbyte, int endbyte)
{
	int i, j;
	int lowbit;  //每個bytes 的 0~3bit
	int highbit; //每個bytes 的 4~7bit

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
