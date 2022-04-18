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
#define FINISHTIME   (double)(6) 
#define N_DATA          (int)( FINISHTIME/SAMPLINGTIME)

/*****************Idl*******************/
int count = 0;

double IntialTime = 0.0;
double DeltTime = 0.0;
double StartTime = 0.0;
double EndTime = 0.0;

/************* 모터 동작 *******************/
int mode = 0;
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

double refTheta = 0.0;
double refPosition = 0.0;

double SamplesingFlag = 0;


/*----------------------------------------------
				     	Main
------------------------------------------------*/

void main()
{

	//determindVcmd(&inputkind);

	DAQmxCreateTask("", &TaskAO);
	DAQmxCreateTask("", &TaskAI);
	DAQmxCreateTask("", &TaskDO);

	CREATE_AI_CH(TaskAI, "Dev6/ai0");
	CREATE_AI_CH(TaskAI, "Dev6/ai1");
	CREATE_AI_CH(TaskAI, "Dev6/ai2");
	CREATE_AO_CH(TaskAO, "Dev6/ao0");
	CREATE_DO_CH(TaskDO, "Dev6/port0");

	for (double i = 0.65; i <= 2.15; i += 0.15) {

		//for (int j = 0; j < 2; j++) {

			//printf("freq=%f[Hz]\n", i);

			DAQmxStartTask(TaskAO);
			DAQmxStartTask(TaskAI);
			DAQmxStartTask(TaskDO);

			RecivedGyromean(&Gyromean);

			WRITE_DO(TaskDO, STOP);
			WRITE_DO(TaskDO, STANDBY);

			MotorIntial_Perpendicular(&mode);
			//MotorIntial(&mode);
			Sleep(1000);

			//if (j == 0) {
			//	magnitude = i;
			//}
			//else {
			//	magnitude = -i;
			//}
			//printf("mag=%f[V]\n", magnitude);

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

					READ_AI(TaskAI, read);

					//LinearX[count] = (read[0] - 2.5) / -2.5 * 12 / 100;
					LinearX[count] = (-6.022 * read[0] + 14.5) / 100.0;
					PotentioDeg[count] = (read[1] - 2.5) * -36;
					GyrooutW[count] = (read[2] - Gyromean) * ConvertW;

					/*----------------------------------------------
								   Process
					-----------------------------------------------*/

					/* Calculate error */
					//errorX[count] = refPosition - LinearX[count];
					//thetaC[count] = PIDcontroller(SAMPLINGTIME, errorX[count], WCOUT, KPOUT, KIOUT, KDOUT);


					/* Outer Loop - Position Control*/
					//errorDeg[count] = thetaC[count] - PotentioDeg[count];
					//errorDeg[count] = 0 - PotentioDeg[count]; //inner loop check

					/* Inner Loop - Angular control*/
					//errorW[count] = KPIN * errorDeg[count];
					//Vcmd[count] = KDIN * (errorW[count] - GyrooutW[count]);

					Vcmd[count] = SinusoidialWave(magnitude, i, Time[count]);
					//outputVcmd(magnitude, i, Time[count], inputkind, &Vcmd[count]);

					/* Linearization */
					//Vss[count] = magnitude;
					Vss[count] = Linearization(Vcmd[count], &mode);

					/* 모터 동작 정지 */
					if (Time[count] >= FINISHTIME) {
						Vss[count] = MotorStop(Time[count], FINISHTIME, &mode);
					}

					//printf("Time = %f[sec]\n", Time[count]);
					//printf("x position = %f[cm] \n", errorX[count]*100);
					//printf("PD output: thetaC = %f[deg] \n", thetaC[count]);
					//printf("Potentio output: Theta = %f[deg] \n", PotentioDeg[count]);
					//printf("After outer controller: errorDeg = %f[deg] \n", errorDeg[count]);
					//printf("After Inner controller: GyrooutW = %f[deg/s] \n", GyrooutW[count]);
					//printf("After Inner controller: errorW = %f [deg/s]\n", errorW[count]);
					//printf("After Inner controller: Vcmd = %f[V], mode = %d \n", Vcmd[count], mode);
					//printf("After Inner controller: Vss = %f[V]\n", Vss[count]);
					//printf("Theta= %f \n\n", PotentioDeg[count]);

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

				MotorIntial_Perpendicular(&mode);
				//MotorIntial(&mode);
				Sleep(1000);

			}

			/*---------------------------------------------------
					 saving the data(in PC)
			-----------------------------------------------------*/


			char name[100] = "sindata_freq";
			WriteTxtFile(name, i, N_DATA, Time, Vcmd, GyrooutW);
			//WriteTxtFile(name, i, N_DATA, Time, PotentioDeg, LinearX);

			//if (j == 0) {
			//	char name[100] = "LCWdata_";
			//	WriteTxtFile(name, i, N_DATA, Time, Vcmd, GyrooutW);
			//}
			//else {
			//	char name[100] = "LCCWdata_";
			//	WriteTxtFile(name, i, N_DATA, Time, Vcmd, GyrooutW);
			//}


			DAQmxStopTask(TaskAO);
			DAQmxStopTask(TaskAI);
			DAQmxStopTask(TaskDO);

		//}
	}

	DAQmxClearTask(TaskAO);
	DAQmxClearTask(TaskAI);
	DAQmxClearTask(TaskDO);

}
#pragma once