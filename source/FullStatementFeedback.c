#include "FullStatementFeedback.h"

void FullStatementFeedback(double r, double K[], double X[], double* uc) {

	*uc = 0;

	for (int i = 0; i < 4; i++) {
		*uc +=  K[i] * X[i];
		printf("feedback=%f\n",*uc);
	}


	*uc = r - *uc;
}



double x_iservo[2] = { 0.0 };
void LQ_servo(double r, double K[], double X[], double* uc){
	
	*uc = 0;

	*uc = r - X[2];

	x_iservo[0] = 0.5 * (*uc + 2 * x_iservo[1]);
	x_iservo[1] = x_iservo[0];
	*uc = -(KISERRVO * 0.005 * 0.5 * (*uc + 2 * x_iservo[1]) + KISERRVO * 0.005 * x_iservo[1]);

	for (int i = 0; i < 4; i++) {
		*uc -= K[i] * X[i];
	}

}



//double Kf[2] = { 0.2775, 5 };
//double Kf[2] = { 0.7500, 4.5000 };
double Kf[3] = { 0.271000000000000, 57.0000000000000, 4000.00000000000 };
//double Kf[3] = { 0.657000000000000, 459.000000000000, 108000.000000000 };
double estimateState[3] = { 0.0 };
double error = 0.0;

void FadingMemoryFilter(double state[], double PredictedState[], double PredictedNextState[]) {


	for (int i = 0; i < 10; i++) {

		error = state[2] - PredictedState[0];

		estimateState[0] = PredictedState[0] + Kf[0] * error;
		estimateState[1] = PredictedState[1] + Kf[1] * error;
		estimateState[2] = PredictedState[2] + Kf[2] * error;

		state[2] = estimateState[0];
		state[3] = estimateState[1];

		PredictedNextState[0] = estimateState[0] + 0.0005 * estimateState[1] + 0.5 * 0.0005 * 0.0005 * estimateState[2];
		PredictedNextState[1] = estimateState[1] + 0.0005 * estimateState[2];
		PredictedNextState[2] = estimateState[2];

		PredictedState[0] = PredictedNextState[0];
		PredictedState[1] = PredictedNextState[1];
		PredictedState[2] = PredictedNextState[2];

	}
}



