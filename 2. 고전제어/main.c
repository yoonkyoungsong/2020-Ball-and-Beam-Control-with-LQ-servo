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
#define FINISHTIME   (double)(25) 
#define N_DATA          (int)( FINISHTIME/SAMPLINGTIME)

/*****************Idl*******************/
int count = 0;

double IntialTime = 0.0;
double DeltTime = 0.0;
double StartTime = 0.0;
double EndTime = 0.0;

/************* 모터 동작 *******************/
int beforemode = 3;
int mode = 3;
double Gyromean = 0;

/*************array initial****************/
double read[4] = { 0 };

double Time[N_DATA] = { 0 };
double Vcmd[N_DATA] = { 0 };
double Wcmd[N_DATA] = { 0 };
double Vss[N_DATA] = { 0 };

double GyrooutW[N_DATA] = { 0 };
double LinearX[N_DATA] = { 0 };
double PotentioDeg[N_DATA] = { 0.0 };

double thetaC[N_DATA] = { 0.0 };

/********controller parameter**********/
static double errorW[N_DATA] = { 0.0 };
static double errorDeg[N_DATA] = { 0.0 };
static double errorX[N_DATA] = { 0.0 };


double PID = 0;

double cmd;
char key = '0';
char beam = '0';

double freq = 1;
int inputkind = 0;
double magnitude = 2.5;

double SamplesingFlag = 0;

/********reference parameter**********/
double refTheta = 30.0 / 180 * M_PI;
double refPosition = 0.0;
double refPosition1 = 0.1;
double refPosition2 = 0.05;
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

			//if (Time[count] < 5) {
			//	refPosition = 0.0;
			//}
			//else if (Time[count] >= 5 && Time[count] < 10) {
			//	refPosition = 0.05;
			//}
			//else if (Time[count] >= 10 && Time[count] <15) {
			//	refPosition = 0.0;
			//}
			//else if (Time[count] >= 15 && Time[count] < 20) {
			//	refPosition = -0.05;
			//}
			//else if (Time[count] >= 20 && Time[count] < 25) {
			//	refPosition = 0.0;
			//}
			//else if (Time[count] >= 25 && Time[count] < 30) {
			//	refPosition = 0.1;
			//}
			//else if (Time[count] >= 30 && Time[count] < 35) {
			//	refPosition = 0.0;
			//}
			//else if (Time[count] >= 35 && Time[count] < 40) {
			//	refPosition = -0.1;
			//}
			//else if (Time[count] >= 40 && Time[count] < 50) {
			//	refPosition = 0.0;
			//}
			
			referencePosition(Time[count], refPosition1, refPosition2, refFreq, &refPosition);
			//printf("time = %f[sec] ,ref = %f[cm]\n",Time[count], refPosition*100);

			/*----------------------------------------------
						 Import_Data
			------------------------------------------------*/

			READ_AI(TaskAI, read);

			//LinearX[count] = (read[0] - 2.5) / -2.5 * 12 / 100;
			LinearX[count] = (-6.022 * read[0] + 14.5) / 100.0;
			PotentioDeg[count] = (read[1] - 2.5) * -36 / 180 * M_PI;
			GyrooutW[count] = (read[2] - Gyromean) * ConvertW / 180 * M_PI;

			/*----------------------------------------------
						   Process
			-----------------------------------------------*/

			
			if ((count >= 1) && (LinearX[count] - LinearX[count - 1] > 0.02)) {
				LinearX[count] = LinearX[count - 1];
			}


			/* Calculate error */
			errorX[count] = refPosition - LinearX[count];
			thetaC[count] = PIDcontroller(SAMPLINGTIME, errorX[count], WCOUT, KPOUT, KIOUT, KDOUT);


			/* Outer Loop - Position Control*/
			errorDeg[count] = thetaC[count] - PotentioDeg[count];
			//errorDeg[count] = refTheta - PotentioDeg[count]; //inner loop check

			/* Inner Loop - Angular control*/
			errorW[count] = KPIN * errorDeg[count];
			Vcmd[count] = KDIN * (errorW[count] - GyrooutW[count]);

			/* Linearization */
			Vss[count] = Linearization(Vcmd[count], &mode);
			changeMode(&beforemode, &mode);

			/* 모터 동작 정지 */
			if (Time[count] >= FINISHTIME) {
				Vss[count] = MotorStop(Time[count], FINISHTIME, &mode);
			}

			//printf("Time = %f[sec]\n", Time[count]);
			//printf("x position = %f[cm] \n",LinearX[count]*100);
			//printf("error x position = %f[cm] \n", errorX[count]*100);
			//printf("PD output: thetaC = %f[deg] \n", thetaC[count]);
			//printf("Potentio output: Theta = %f[deg] \n", PotentioDeg[count]);
			//printf("After outer controller: errorDeg = %f[deg] \n", errorDeg[count]);
			//printf("After Inner controller: GyrooutW = %f[deg/s] \n", GyrooutW[count]);
			//printf("After Inner controller: errorW = %f [deg/s]\n", errorW[count]);
			//printf("After Inner controller: Vcmd = %f[V], mode = %d \n", Vcmd[count], mode);
			//printf("After Inner controller: Vss = %f[V]\n", Vss[count]);
			//printf("Theta= %f \n\n", PotentioDeg[count]);
			//printf("\n");

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

		//MotorIntial_Perpendicular(&mode);
		MotorIntial(&mode);
		Sleep(1000);

	}

	/*---------------------------------------------------
			 saving the data(in PC)
	-----------------------------------------------------*/

	char name1[100] = "refpulse_Vcmd_Gyro";
	char name2[100] = "refpulse_deg_pos";
	WriteTxtFile(name1, 2, N_DATA, Time, Vcmd, GyrooutW);
	WriteTxtFile(name2, 2, N_DATA, Time, PotentioDeg, LinearX);


	DAQmxStopTask(TaskAO);
	DAQmxStopTask(TaskAI);
	DAQmxStopTask(TaskDO);


	DAQmxClearTask(TaskAO);
	DAQmxClearTask(TaskAI);
	DAQmxClearTask(TaskDO);

}
#pragma once