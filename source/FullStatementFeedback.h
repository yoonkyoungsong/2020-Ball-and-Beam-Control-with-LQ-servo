#ifndef HEADER_USER_DEFINED_FSF
#define HEADER_USER_DEFINED_FSF

#include "userHeader.h"

/*Full statement feedback gain*/
//#define K0 (double) (21.823646644991314)
//#define K1 (double) (0.067301278216949)
//#define K2 (double) (44.732307609641340)
//#define K3 (double) (19.181978339613750)


///*LQ servo*/
//#define K0 (double) (23.107655218595703)
//#define K1 (double) (0.514468752152275)
//#define K2 (double) (1.093327890071155e+02)
//#define K3 (double) (26.124990858635390)
//
//#define KISERRVO (double) (-1.000000000000003e+02)

///*LQ servo*/
//#define K0 (double) (26.513037252858716)
//#define K1 (double) (0.671959896501992)
//#define K2 (double) (1.413652647024647e+02)
//#define K3 (double) (31.676444373919768)
//
//#define KISERRVO (double) (-1.581138830084190e+02)

/*LQ servo*/
//#define K0 (double) (26.513037252858716)
//#define K1 (double) (0.671959896501992)
//#define K2 (double) (1.413652647024647e+02)
//#define K3 (double) (31.676444373919768)
//
//#define KISERRVO (double) (-1.581138830084190e+02)

/*LQ servo*/
#define K0 (double) (32.0642280816676)
#define K1 (double) (0.781961881167815)
#define K2 (double) (125.149534678065)
#define K3 (double) (40.9590444455449)

#define KISERRVO (double) (-141.421356237309)

void FadingMemoryFilter(double state[], double PredictedState[], double PredictedNextState[]);
void FullStatementFeedback(double r, double K[], double X[], double* uc);
void LQ_servo(double r, double K[], double X[], double* uc);

#endif
#pragma once
