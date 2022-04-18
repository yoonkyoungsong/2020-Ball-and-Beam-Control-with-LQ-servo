#ifndef SOURCE_USER_DEFINED_FUNCTION
#define SOURCE_USER_DEFINED_FUNCTION

//		

#include "Function.h"


double CheckwindowsTime(void)
{
	LARGE_INTEGER  liCount, liFreq;

	QueryPerformanceCounter(&liCount);
	QueryPerformanceFrequency(&liFreq);

	return((liCount.QuadPart / ((double)(liFreq.QuadPart))) * 1000.0);


}


/********************idle time 함수***********************/

/* Parameter */
double buff[2];
#define SAMPLINGTIME (double)(0.005) 


/************* 텍스트파일 저장하기 ***************/

/* Fumction */
void WriteTxtFile(char Name[], double k, double N_DATA, double data1[], double data2[], double data3[]) {

	FILE* pF1;  //task 시작 시간 data  // header 파일 만들기 

	char s1[20];
	sprintf(s1, "%.2f", k);
	char s2[20] = ".txt";
	//char way[1000] = "C:\\Users\\xde12\\OneDrive\\바탕 화면\\";

	strcat(Name, s1);
	strcat(Name, s2);


	pF1 = fopen(Name, "w");

	for (int count = 0; count < N_DATA; count++)
	{
		fprintf(pF1, "%f %f %f \n", data1[count], data2[count], data3[count]);
	}

	fclose(pF1);

	printf("Storage data file\n\n");
}


#endif