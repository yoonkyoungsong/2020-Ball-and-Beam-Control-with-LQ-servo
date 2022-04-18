#ifndef HEADER_USER_DEFINED_DCMS
#define HEADER_USER_DEFINED_DCMS

/************************************/

#include "userHeader.h"
#include "userDAQ.h"

/************define Parameter *****************/

#define STANDBY (int) 3
#define STOP	(int) 0
#define CW		(int) 2
#define CCW		(int) 1

#define RESOLUTION		(double) 0.2


#define GO (double)(3) // sinusoid 기본조건
//#define STOP (double)(2)

#define MotorStandard  (double)(2.5)
#define ConvertW (double)(1492.53731)


#define KP_T (double)(10)

/* 고전제어 되는 값*/

////inner wc =38
#define WCIN  (double)(38.0)
#define KPIN  (double)(62.232437371391790) 
#define KIIN  (double)(0.0)
#define KDIN  (double)(0.824119873546366)

////outer wc = 5 => PD controller
#define WCOUT  (double)(5.0)
#define KPOUT  (double)(3.571428571428572) 
#define KIOUT  (double)(0.0)
#define KDOUT  (double)(1.0)

//outer wc = 4 => PD controller
#define WCOUT  (double)(4.5)
#define KPOUT  (double)(2.892857142857143) 
#define KIOUT  (double)(0.0)
#define KDOUT  (double)(0.900000000000000)



/* Parameter */
//  -5 <= Vcmd <= 5
#define CWA  (double)(0.019510365725417)
#define CWB  (double)(0.210984917277933)
#define CWC  (double)(1.289629949929679)

#define CCWA  (double)(0.027563003034701)
#define CCWB  (double)(0.182739609447157)
#define CCWC  (double)(1.283664579759863)





/************************************/
/************************************/

double MotorStop(double Time, double FINISHTIME, int* mode);
void BeamDirectionCheck();
void CWBeamDirectionCheck(int* mode);
void CCWBeamDirectionCheck(int* mode);
void MotorIntial(int*mode);
void MotorIntial_Perpendicular(int* mode);
void RecivedGyromean(double* Gyromean);

void determindVcmd(int* inputkind);
void outputVcmd(double magnitude, double freq, double time, int inputkind, double* Vcmd);

void referencePosition(double time, double ref1, double ref2, double f, double* ref);

double Linearization(double Vcmd, int* Linearmode);
double PIDcontroller(double SamplingTime, double input, double Wc, double Kp, double Ki, double Kd);




#endif
#pragma once

