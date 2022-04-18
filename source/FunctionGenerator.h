#ifndef HEADER_USER_DEFINED_FUNCTIONGENERATOR
#define HEADER_USER_DEFINED_FUNCTIONGENERATOR

#include "userHeader.h"


double SinusoidialWave(double magnitude, double Frequency, double time);
double TriangulerWave(double magnitude, double Tri_time, double CurrentTime_tri);
double PulseWave(double magnitude, double pluse_time, double time);
double Lamp(double Vcmd);
double angularCheck(double Vcmd, double degree);

#endif



#pragma once
