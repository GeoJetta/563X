/*	A RobotC sensor logging and replay program.
 *  Copyright (C) 2016  Geoffrey CJ Smith

 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.

 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.

 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>. */


//change to sensor names
#define LEFT_DRIVE_ENC I2C_1 //FIXME: find and replace all of these with those currently in use
#define RIGHT_DRIVE_ENC I2C_2
#define LEFT_ARM_ENC I2C_4
#define RIGHT_ARM_ENC I2C_3
#define GYRO_SENSE Gyro

//wait until all motors have gone to target
void waitForMoveFinish()
{

	

}

//control recording with outside sources
bool recording = false;
//time between sampling of sensor values (decreases resolution, but also decreases file size)
int sampleTime = 10;

//recording task
task Record()
{

	//Create arrays for all sensors
	DynamicArray LeftDrive;
	DynamicArray RightDrive;
	DynamicArray LeftArm;
	DynamicArray RightArm;
	DynamicArray Gyro_Array;

	//Init arrays for all sensor
	DynamicArrayInit(&LeftDrive);
	DynamicArrayInit(&RightDrive);
	DynamicArrayInit(&LeftArm);
	DynamicArrayInit(&RightArm);
	DynamicArrayInit(&Gyro_Array);

	//amount of allowable deadzone in sensor changes between cycles (decreases resolution, but also decreases file size)
	int err = 10;
	//counter
	int i = 0;

	writeDebugStreamLine("slaveMotor(L3, L2); slaveMotor(L1, L2); slaveMotor(L4, L2);");
	writeDebugStreamLine("slaveMotor(R3, R2); slaveMotor(R1, R2); slaveMotor(R4, R2);");


	while (!recording)
	{

		wait1Msec(10);

	}

	while (recording)
	{

		//If sensor values outside of deadzone for this iteration
		if (DynamicArrayGet(&LeftDrive, i-1) < SensorValue[LEFT_DRIVE_ENC] - err || DynamicArrayGet(&LeftDrive, i-1) > SensorValue[LEFT_DRIVE_ENC] + err)
		{

			//Add value to array
			DynamicArraySet(&LeftDrive, i, SensorValue[LEFT_DRIVE_ENC]);

		}
		if (DynamicArrayGet(&RightDrive, i-1) < SensorValue[RIGHT_DRIVE_ENC] - err || DynamicArrayGet(&RightDrive, i-1) > SensorValue[RIGHT_DRIVE_ENC] + err)
		{

			DynamicArraySet(&RightDrive, i, SensorValue[RIGHT_DRIVE_ENC]);

		}
		if (DynamicArrayGet(&LeftArm, i-1) < SensorValue[LEFT_ARM_ENC] - err || DynamicArrayGet(&LeftArm, i-1) > SensorValue[LEFT_ARM_ENC] + err)
		{

			DynamicArraySet(&LeftArm, i, SensorValue[LEFT_ARM_ENC]);

		}
		if (DynamicArrayGet(&RightArm, i-1) < SensorValue[RIGHT_ARM_ENC] - err || DynamicArrayGet(&RightArm, i-1) > SensorValue[RIGHT_ARM_ENC] + err)
		{

			DynamicArraySet(&RightArm, i, SensorValue[RIGHT_ARM_ENC]);

		}
		if (DynamicArrayGet(&Gyro_Array, i-1) < SensorValue[GYRO_SENSE] - err || DynamicArrayGet(&Gyro_Array, i-1) > SensorValue[GYRO_SENSE] + err)
		{

			DynamicArraySet(&Gyro_Array, i, SensorValue[GYRO_SENSE]);

		}

		i++;

		wait1Msec(sampleTime);

	}


	for (long ii = 0; ii < DynamicArraySize(&LeftDrive); ii++)
	{

		//write array values to debug stream.
		//FIXME: input motor names etc.
		writeDebugStreamLine("setMotorTarget(L, %d);", DynamicArrayGet(&LeftDrive, ii));
		writeDebugStreamLine("setMotorTarget(R, %ld);", DynamicArrayGet(&RightDrive, ii));
		writeDebugStreamLine("setMotorTarget(LEFT_SIDE_ARM, %ld);", DynamicArrayGet(&LeftArm, ii));
		writeDebugStreamLine("setMotorTarget(RIGHT_SIDE_ARM, %ld);", DynamicArrayGet(&RightArm, ii));

		writeDebugStreamLine("wait1MSec(sampleTime+2);");

	}

}
