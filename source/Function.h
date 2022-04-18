
#ifndef HEADER_USER_DEFINED_FUNCTION
#define HEADER_USER_DEFINED_FUNCTION

#include "userHeader.h"

double CheckwindowsTime(void); // check the windows time in [ms]
void WriteTxtFile(char Name[], double k, double N_DATA, double data1[], double data2[], double data3[]);

void IdleTime(double DeltTime, double CurrentTime);

#endif