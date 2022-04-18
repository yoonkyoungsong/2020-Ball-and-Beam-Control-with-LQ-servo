#include "userHeader.h"
#include "userDefineHeader.h"


/************* DAQ ****************/

extern TaskHandle TaskAO;
extern TaskHandle TaskAI;
extern TaskHandle TaskDO;

/*-----------------------------------------------
				   Parameters
-------------------------------------------------*/

/************** Time *******************/
#define SAMPLINGTIME (double)(0.005) 
#define FINISHTIME   (double)(45) 
#define N_DATA          (int)( FINISHTIME/SAMPLINGTIME)

/*****************Idl*******************/
int count = 0;

double IntialTime = 0.0;
double DeltTime = 0.0;
double StartTime = 0.0;
double EndTime = 0.0;

/************* 모터 동작 *******************/
int beforemode = 0;
int mode = 0;
double Gyromean = 0;

/*************array initial****************/
double read[4] = { 0 };

double Time[N_DATA] = { 0 };
double Vcmd[N_DATA] = { 0 };
double Vss[N_DATA] = { 0 };

double GyrooutW[N_DATA] = { 0 };
double LinearX[N_DATA] = { 0 };
double RotaryTheta[N_DATA] = { 0.0 };
double thetaC[N_DATA] = { 0.0 };

static double errorW[N_DATA] = { 0.0 };
static double errorDeg[N_DATA] = { 0.0 };
static double errorX[N_DATA] = { 0.0 };


/********State parameter**********/

double X[4] = { 0.0 };
double Xdot[4] = { 0.0 };

double theta[N_DATA] = { 0.0 };
double w[N_DATA] = { 0.0 };
double x[N_DATA] = { 0.0 };
double x_dot[N_DATA] = { 0.0 };

double state[4] = { 0.0 };
double PredictedState[3] = { 0.0 };
double PredictedNextState[3] = { 0.0 };

/********controller parameter**********/
double K[4] = { K0, K1, K2, K3 };
char key = '0';


/********reference parameter**********/
double refTheta = 0.0;
double refPosition = 0.0;
double refPosition1 = 0.05;
double refPosition2 = 0.1;
double refFreq = 0.2;

/*----------------------------------------------
						Main
------------------------------------------------*/

void main()
{

	DAQmxCreateTask("", &TaskAO);
	DAQmxCreateTask("", &TaskAI);
	DAQmxCreateTask("", &TaskDO);

	CREATE_AI_CH(TaskAI, "Dev6/ai0");
	CREATE_AI_CH(TaskAI, "Dev6/ai1");
	CREATE_AI_CH(TaskAI, "Dev6/ai2");
	CREATE_AO_CH(TaskAO, "Dev6/ao0");
	CREATE_DO_CH(TaskDO, "Dev6/port0");

	 
	DAQmxStartTask(TaskAO);
	DAQmxStartTask(TaskAI);
	DAQmxStartTask(TaskDO);

	RecivedGyromean(&Gyromean);

	WRITE_DO(TaskDO, STOP);
	WRITE_DO(TaskDO, STANDBY);


	MotorIntial(&mode);
	Sleep(1000);
	printf("Ready to Start \n");
	printf("If you wnat to start, press the Enterkey \n");
	key = getch();


	if (key == 13) {

		StartTime = CheckwindowsTime() * 0.001;

		do {

			IntialTime = CheckwindowsTime() * 0.001;
			Time[count] = IntialTime - StartTime;

			/*----------------------------------------------
						 Import_Data
			------------------------------------------------*/

			referencePosition(Time[count], refPosition1, refPosition2, refFreq, &refPosition);
			//printf("time = %f[sec] ,ref = %f[cm]\n",Time[count], refPosition*100);
			
			READ_AI(TaskAI, read);

			//LinearX[count] = (read[0] - 2.5) / -2.5 * 12 / 100;
			LinearX[count] = (-6.022 * read[0] + 14.5) / 100.0;
			RotaryTheta[count] = (read[1] - 2.5) * -36;
			GyrooutW[count] = (read[2] - Gyromean) * ConvertW;
			
			state[0] = RotaryTheta[count] / 180 * M_PI;
			state[1] = GyrooutW[count] / 180 * M_PI;
			state[2] = LinearX[count];

			/*----------------------------------------------
						   Process
			-----------------------------------------------*/

			/* Predicted State Xdot */
			FadingMemoryFilter(state, PredictedState, PredictedNextState);
			x[count] = state[2];
			x_dot[count] = state[3];

			/* Full-state */
			//FullStatementFeedback(Vref, K, state, &Vcmd[count]);
			LQ_servo(refPosition, K, state, &Vcmd[count]);

			/* Linearization */
			Vss[count] = Linearization(Vcmd[count], &mode);
			changeMode(&beforemode, &mode);

			/* 모터 동작 정지 */
			if (Time[count] >= FINISHTIME) {
				Vss[count] = MotorStop(Time[count], FINISHTIME, &mode);
			}


			/*----------------------------------------------
							Export_Data
			------------------------------------------------*/

			WRITE_AO(TaskAO, Vss[count]);
			WRITE_DO(TaskDO, mode);

			/*----------------------------------------------
							Idle Time
			------------------------------------------------*/


			while (1) {
				DeltTime = CheckwindowsTime() * 0.001;
				EndTime = DeltTime - IntialTime;
				if (EndTime >= SAMPLINGTIME) {
					break;
				}

			}

			count++;

		} while (SAMPLINGTIME * ((double)(count)) < FINISHTIME);


		count = 0;

		WRITE_DO(TaskDO, STANDBY);
		Sleep(1000);

		MotorIntial(&mode);
		Sleep(1000);


		/*---------------------------------------------------
				 saving the data(in PC)
		-----------------------------------------------------*/

		char name1[100] = "LQservo_Vcmd_x";
		char name2[100] = "LQservo_theta_omega";
		char name3[100] = "LQservo_state";
		WriteTxtFile(name1, 3, N_DATA, Time, Vcmd, LinearX);
		WriteTxtFile(name2, 3, N_DATA, Time, RotaryTheta, GyrooutW);
		WriteTxtFile(name3, 3, N_DATA, LinearX, x, x_dot);

	}

	DAQmxStopTask(TaskAO);
	DAQmxStopTask(TaskAI);
	DAQmxStopTask(TaskDO);

	DAQmxClearTask(TaskAO);
	DAQmxClearTask(TaskAI);
	DAQmxClearTask(TaskDO);

}
#pragma once