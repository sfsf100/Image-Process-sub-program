#include "image.h"

void swap(int *a, int *b)
{
	int tmp = *a;
	*a = *b;
	*b = tmp;
}

int parition(int *arr, int front, int end)
{
	int j;
	int i = front - 1;
	int pivot = arr[end];
	for (j = front; j<end; j++)
	{
		if (arr[j]<pivot)
		{
			i++;
			swap(&arr[i], &arr[j]);
		}
	}
	i++;
	swap(&arr[i], &arr[end]); //j++大小比較執行完 最後根交換 
	return i; // *因為遞迴 所以要 
}

void quicksort(int *arr, int front, int end)
{
	if (front<end) //注意 
	{
		int pivot = parition(arr, front, end); //recursive
		quicksort(arr, front, pivot - 1);  //right
		quicksort(arr, pivot + 1, end);    //left
	}
}


void Median_filter(Image img, Image &median)
{
	//median = Mat::zeros(img.size(), CV_16U);
	int arr[121] = { 0 };
	int block_size = 11;// 11 x 11
	int ker_size = (block_size-1)/2;// 11 x 11
	int count = 0;
	float result;
	for (int j = ker_size; j < img.height - ker_size; j++)	{
		for (int i = ker_size; i < img.width - ker_size; i++){
			count = 0;
			for (int a = -ker_size; a <= ker_size; a++){
				for (int b = -ker_size; b <= ker_size; b++){
					arr[count] = img.get_pixel( i + b, j + a, 0)*255;
					//printf("%d = %d\n", arr[count], count);
					count++;
				}
			}
			quicksort(arr, 0, 121 - 1);//121
			//printf("%d\n", arr[60]);
			result = arr[60];
			median.set_pixel(i, j, 0, result /255);
		}
	}
}