/***************** 헤더파일 불러오기 *******************/
#include "DigitalControl.h"
#include "FunctionGenerator.h"

/***************모터 동작 정지*******************/
	
/* Parameter */
double Vss;

/* Fumction */
double MotorStop(double Time, double FINISHTIME, int* mode) {

	Vss = 2.5;
	*mode = STANDBY;

	return Vss;
}

/************ 모터 초기 위치 설정 함수 ****************/

/* Parameter */

double read[3];
double cmd;
TaskHandle TaskAO;
TaskHandle TaskAI;
TaskHandle TaskDO;

/* Fumction */
void MotorIntial(int* mode) {

	//Gimbal 가운데 맞춤


	while (1) {


		READ_AI(TaskAI, read);
		WRITE_AO(TaskAO, 1.5);

		if (read[1] < 2.5) {  //2.57
			WRITE_DO(TaskDO, CCW);
		}
		else if (read[1] > 2.5) {
			WRITE_DO(TaskDO, CW);
		}

		if ((read[1] > 2.47) && (read[1] <= 2.53)) {
			break;
		}

		//printf("Initial Voltage = %f \n", read[1]);
	}


	printf("Initial Voltage = %f \n", read[1]);

	*mode = STANDBY;
	WRITE_AO(TaskAO, 2);
	WRITE_DO(TaskDO, *mode);
	
	
}

void MotorIntial_Perpendicular(int* mode) {

	//Gimbal 가운데 맞춤


	while (1) {


		READ_AI(TaskAI, read);
		WRITE_AO(TaskAO, 1.5);

		if (read[1] < 2.5) {  //2.57
			WRITE_DO(TaskDO, CW);
		}
		else if (read[1] > 2.5) {
			WRITE_DO(TaskDO, CCW);
		}

		if ((read[1] > -0.05) && (read[1] <= 0.05)) {
			break;
		}

		//printf("Initial Voltage = %f \n", read[1]);
	}


	printf("Initial Voltage = %f \n", read[1]);

	*mode = STANDBY;
	WRITE_AO(TaskAO, 2);
	WRITE_DO(TaskDO, *mode);


}

/************ 빔 초기 위치 설정 함수 ****************/


void BeamDirectionCheck() {

	int i = 0;
	char key = ' ';


	printf("If Beam is opposite, press the 'r' that is CW or 'l'is CCW. Otherwise, press any key.\n");
	WRITE_AO(TaskAO, 2);

	key = getch();
	if (key == 'r') {

		do {
			WRITE_DO(TaskDO, CCW);
			i++;
		} while (i < 300000);

	}
	else if(key == 'l') {

		do {
			WRITE_DO(TaskDO, CW);
			i++;
		} while (i < 30000);

	}
	//else {
	//	do {
	//		write_do(taskdo, standby);
	//		i++;
	//	} while (i < 3);

	//}

}

void CWBeamDirectionCheck(int *mode) {

	int i = 0;
	char key= ' ';


	printf("If Beam is opposite, press the 'r'. Otherwise, press any key.\n");
	WRITE_AO(TaskAO, 2);

	key = getch();
	if (key == 'r') {

		do {
			WRITE_DO(TaskDO, CW);
			i++;
		} while (i < 30000);

		MotorIntial(*mode);
	}
	else {
		do {
			WRITE_DO(TaskDO, CW);
			i++;
		} while (i < 3);
		MotorIntial(*mode);
	
	}

}

void CCWBeamDirectionCheck(int* mode) {

	int i = 0;
	char key = ' ';


	printf("If Beam is opposite, press the 'r'. Otherwise, press any key.\n");
	WRITE_AO(TaskAO, 2);

	key = getch();
	if (key == 'r') {

		do {
			WRITE_DO(TaskDO, CCW);
			i++;
		} while (i < 300000);

		MotorIntial(*mode);
	}
	else {
		do {
			WRITE_DO(TaskDO, CCW);
			i++;
		} while (i < 3);
		MotorIntial(*mode);

	}

}

/**************** 자이로 평균값 받기  ********************/
void RecivedGyromean(double* Gyromean) {
	
	int i = 0;


	do {

		READ_AI(TaskAI, read);

		WRITE_AO(TaskAO, 2);
		WRITE_DO(TaskDO, STANDBY);
			
		if (i == 0) {
			*Gyromean = read[2];
		}
		else {
			*Gyromean = (*Gyromean + read[2]) / 2;
		}
	
		i++;

	} while (i < 100);

	*Gyromean = *Gyromean;
	printf("Gyromean = %f\n", *Gyromean);

}

/**************** 입력 정하는 함수  ********************/
void determindVcmd(int* inputkind) {

	printf("/*-----------------------------------------------------------\n");
	printf("Select Vcmd(input), press the int number.\n");
	printf("1. static motor characterisitic Vcmd\n");
	printf("2. sinusodial wave\n");
	printf("3. triangluar wave\n");
	printf("4. step wave\n");
	printf("-----------------------------------------------------------*/\n");

	scanf("%d", &*inputkind);

}

/**************** 정한 입력 내보내는 함수  ********************/
void outputVcmd(double magnitude, double freq, double time, int inputkind, double* Vcmd) {

	switch (inputkind) {

		case 1 : 
			*Vcmd = magnitude;
			break;

		case 2 :
			*Vcmd = SinusoidialWave(magnitude, freq, time);
			break;
		
		case 3:
			*Vcmd = TriangulerWave(magnitude, freq, time);
			break;
		
		case 4:
			*Vcmd = 1;
			break;	
	
	}

}

/**************** reference 입력 출력하는 함수 ********************/

void referencePosition(double time, double ref1, double ref2, double f, double* ref) {

	if (time < 1/f) {
		*ref = 0.0;
	}
	else if (time >= 1 / f && time < 2 / f) {
		*ref = ref1;
	}
	else if (time >= 2 / f && time < 3 / f) {
		*ref = 0.0;
	}
	else if (time >= 3 / f && time < 4 / f) {
		*ref = -1*ref1;
	}
	else if (time >= 4 / f && time < 5 / f) {
		*ref = 0.0;
	}
	else if (time >= 5 / f && time < 6 / f) {
		*ref = ref2;
	}
	else if (time >= 6 / f && time < 7 / f) {
		*ref = 0.0;
	}
	else if (time >= 7 / f && time < 8 / f) {
		*ref = -1*ref2;
	}
	else if (time >= 8 / f && time < 10 / f) {
		*ref = 0.0;
	}

}


/**************** 선형화 함수  ********************/

double Linearization(double Vcmd, int* mode) {
	
	double Vss = 0;

	if (Vcmd < -5) {
		Vcmd = 5;
		Vss = (CCWA * Vcmd * Vcmd + CCWB * (Vcmd) + CCWC);
		*mode = CCW;
	}
	else if (Vcmd <= -0.5 && Vcmd > -5) {
		Vcmd = -1 * Vcmd;
		Vss = CCWA * Vcmd * Vcmd + CCWB * (Vcmd)+CCWC;
		*mode = CCW;
	}
	else if (Vcmd >= -0.5 && Vcmd < 0.5) {
		Vss = 0;
		*mode = STANDBY;
	}
	else if (Vcmd >= 0.5 && Vcmd < 5) {
		Vss = CWA * Vcmd * Vcmd + CWB * (Vcmd)+CWC;
		*mode = CW;
	}
	else if ( Vcmd >= 5) {
		Vcmd = 5;
		Vss = CWA * Vcmd * Vcmd + CWB * (Vcmd)+CWC;
		*mode = CW;
	}

	return Vss;
}

/**************** CCW/CW 변환 flag  ********************/
void changeMode(int* beforemode, int* mode) {
	if ((*mode != STANDBY) && (*beforemode != STANDBY) && (*beforemode != *mode)) {
		*mode = STANDBY;
	}
	*beforemode = *mode; 
}


/**************** PID Controllor ********************/

/* Parameter */

double PID;
double x_i[2] = { 0.0 };
double x_d[2] = { 0.0 };
double P[2] = { 0.0 };
double I[2] = { 0.0 };
double D[2] = { 0.0 };
double Input[2] = { 0.0 };

/* Fumction */
double PIDcontroller(double SamplingTime, double input, double Wc, double Kp, double Ki, double Kd)
{
	Input[0] = input;

	P[0] = Kp * Input[0];
	//I[0] = I[1]  + Ki * SamplingTime / 2 * Input[0] + Ki*SamplingTime / 2 * Input[1];

	x_i[0] = 0.5 * (Input[0] + 2 * x_i[1]);
	x_i[1] = x_i[0];
	I[0] = Ki * SamplingTime * 0.5 * (Input[0] + 2 * x_i[1]) + Ki * SamplingTime * x_i[1];


	x_d[0] = ((Input[0] - (10 * Wc * SamplingTime - 2) * x_d[1]) / (10 * Wc * SamplingTime + 2));
	x_d[1] = x_d[0];
	D[0] = (20 * Kd * Wc * Input[0]) / (10 * Wc * SamplingTime + 2) - (10 * Wc * SamplingTime - 2) * 20 * Wc * Kd * x_d[0] / (10 * Wc * SamplingTime + 2) - 20 * Wc * Kd * x_d[0];
	
	//printf("P present: %f, past: %f\n", P[0], P[1]);
	//printf("I present: %f, past: %f\n", I[0], I[1]);
	//printf("D present: %f, past: %f\n\n", D[0], D[1]);

	P[1] = P[0];
	I[1] = I[0];
	D[1] = D[0];
	Input[1] = Input[0];

	PID = P[0] + I[0] + D[0];

	return PID;
}





 
