#define CLOSED 1
#define OPEN 0
#define ENCODERMODE 1
#define GYROMODE 2

const int tsArray[128] = {
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 22, 23, 24, 24,
		25, 25, 25, 25, 26, 27, 27, 28, 28, 28,
		28, 29, 30, 30, 30, 31, 31, 32, 32, 32,
		33, 33, 34, 34, 35, 35, 35, 36, 36, 37,
		37, 37, 37, 38, 38, 39, 39, 39, 40, 40,
		41, 41, 42, 42, 43, 44, 44, 45, 45, 46,
		46, 47, 47, 48, 48, 49, 50, 50, 51, 52,
		52, 53, 54, 55, 56, 57, 57, 58, 59, 60,
		61, 62, 63, 64, 65, 66, 67, 67, 68, 70,
		71, 72, 72, 73, 74, 76, 77, 78, 79, 79,
		80, 81, 83, 84, 84, 86, 86, 87, 87, 88,
		88, 89, 89, 90, 90, 90, 90, 90
};

int lClawTarget = SensorValue[LeftClawPot];
int rClawTarget = SensorValue[RightClawPot];
int liftTarget = (SensorValue[LeftMiddleLiftEnc] + SensorValue[RightMiddleLiftEnc])/2;
int lDrivePTarget = SensorValue[LeftBackDriveEnc];
int rDrivePTarget = SensorValue[LeftBackDriveEnc];
int driveAnglePTarget = SensorValue[gyro];

int driveFeedbackSource = ENCODERMODE;

bool doClawHold = false;
bool doLiftHold = false;
bool doDriverDrive = true;
bool doDriveP = false;

int clipValue( int value, int max = 127 )
{

	if( value > max ) value = max;
	else if( value < -max ) value = -max;

	return value;

}

int trueSpeed( int iInput )
{

	return sgn(iInput)*tsArray[abs(clipValue(iInput))];

}

bool completed = false;

bool waitUntilComplete()
{

	while( !completed )
	{

		wait10Msec(10);

	}

	return true;

}

void claw( int pwr )
{

	motor[LeftClaw] = pwr;
	motor[RightClaw] = pwr;

}

void drive( int lpwr, int rpwr = lpwr )
{

	clipValue(lpwr, 90);
	clipValue(rpwr, 90);

	if( abs(lpwr) > 15 )
	{

		motor[LeftBottomDrive] = lpwr;
		motor[LeftTopDrive] = lpwr;

	}
	if( abs(rpwr) > 15 )
	{

		motor[RightBottomDrive] = rpwr;
		motor[RightTopDrive] = rpwr;

	}
	else
	{

		motor[RightBottomDrive] = 0;
		motor[RightTopDrive] = 0;
		motor[LeftBottomDrive] = 0;
		motor[LeftTopDrive] = 0;

	}

}

task drivePControl()
{

	int lErr;
	int rErr;

	while( true )
	{

		while( doDriveP )
		{

			completed = false;

			switch( driveFeedbackSource )
			{

				case ENCODERMODE:

					lErr = (lDrivePTarget - SensorValue[LeftBackDriveEnc])/4;
					rErr = (rDrivePTarget + SensorValue[RightBackDriveEnc])/4;

				break;

				case GYROMODE:

					lErr = (-driveAnglePTarget + SensorValue[gyro])/2;
					rErr = -lErr;

				break;

				default:



				break;

			}

			if( driveFeedbackSource == 2 && driveAnglePTarget < 0 )
			{

				motor[LeftBottomDrive] = motor[LeftTopDrive] = trueSpeed(lErr);
				motor[RightBottomDrive] = motor[RightTopDrive] = trueSpeed(rErr);

			}
			else
			{
				motor[LeftBottomDrive] = motor[LeftTopDrive] = trueSpeed(lErr);
				motor[RightBottomDrive] = motor[RightTopDrive] = trueSpeed(rErr);
			}

			if( abs( lErr ) < 20 && abs( rErr ) < 20 )
				completed = true;

			wait1Msec( 15 );

		}

		wait1Msec( 5 );

	}

}
//Set current target (absolute movement) (no inputs gives reset to current sensor values)
void setDriveTarget( int lTarget = SensorValue[LeftBackDriveEnc], int rTarget = ( lTarget == SensorValue[LeftBackDriveEnc] ) ? -SensorValue[RightBackDriveEnc] : lTarget )
{

	lDrivePTarget = lTarget;
	rDrivePTarget = rTarget;

	driveFeedbackSource = ENCODERMODE;

}

void moveDriveTarget( int lTarget, int rTarget = lTarget ) //add or subtract from the current target (relative movement)
{

	//setDriveTarget(); //Set targets to current encoder values

	lDrivePTarget += lTarget;
	rDrivePTarget += rTarget;

}

void setGyroTarget( int angleTarget ) //make turns by setting the drive targets toward a gyro angle (absolute movement)
{

	setDriveTarget(); //Set targets to current encoder values

	driveFeedbackSource = GYROMODE; //Use gyro as feedback source for turns

	driveAnglePTarget = angleTarget;

	//setDriveTarget(); //Set targets to current encoder values

}

void moveGyroTarget( int angleTarget ) //make turns by moving the drive targets toward a gyro angle (relative movement)
{

	setDriveTarget(); //Set targets to current encoder values

	driveFeedbackSource = GYROMODE; //Use gyro as feedback source for turns

	driveAnglePTarget += angleTarget;

	setDriveTarget(); //Set targets to current encoder values

}

void autonLift( int pwr )
{

	motor[LeftBottomLift] = pwr;
	motor[RightBottomLift] = pwr;
	motor[MiddleLift] = pwr;
	motor[TopLift] = pwr;

}

void lift( int pwr )
{

	if(vexRT[Btn7UXmtr2])
	{

		doDriverDrive = false;
		clearTimer(T1);
		while(time1[T1] < 750)
		{

			drive(127, 127);

		}
		drive(0);
		pwr = -127;

	}

	doDriverDrive = true;

	if( vexRT[Btn8RXmtr2] )
	{

		doLiftHold = true;
		doClawHold = true;

			liftTarget = 1000;
			lClawTarget = 200;
			rClawTarget = 960;

	}
	else if( vexRT[Btn8LXmtr2] )
	{

		doLiftHold = true;
		doClawHold = true;

			liftTarget = 800;
			lClawTarget = 610;
			rClawTarget = 1300;
	}
	else
	{

		if( !vexRT[Btn8UXmtr2] && !vexRT[Btn8DXmtr2] )
		{
			doLiftHold = false;
			doClawHold = false;

		}

	}

	motor[LeftBottomLift] = pwr;
	motor[RightBottomLift] = pwr;
	motor[MiddleLift] = pwr;
	motor[TopLift] = pwr;

}

void driverDrive( int lpwr, int rpwr )
{

	if( doDriverDrive )
	{

			motor[RightBottomDrive] = rpwr;
			motor[RightTopDrive] = rpwr;
			motor[LeftBottomDrive] = lpwr;
			motor[LeftTopDrive] = lpwr;

	}

}


task clawHold()
{

	int lErr;
	int rErr;

	while( true )
	{

		while( doClawHold )
		{

			lErr = (lClawTarget - SensorValue[LeftClawPot])/3;
			rErr = (rClawTarget - SensorValue[RightClawPot])/3;


			motor[LeftClaw] = -clipValue( lErr );

			motor[RightClaw] = -clipValue( rErr );

			wait1Msec( 15 );

		}

		wait1Msec( 5 );

	}

}

task liftHold()
{

	int err;

	while( true )
	{

		while( doLiftHold )
		{

			err = (liftTarget - ( -SensorValue[LeftMiddleLiftEnc] + SensorValue[RightMiddleLiftEnc] ) / 2 );

			autonLift( clipValue( err ) );

			wait1Msec( 15 );

		}

		wait1Msec( 5 );

	}

}

void driverClaw( int lpwr, int rpwr = lpwr )
{

	if( SensorValue[LeftClawPot] < 1550 || vexRT[Btn6UXmtr2] )
	{

		motor[ LeftClaw ] = lpwr;

	}
	else
	{

		motor[ LeftClaw ] = 10;

	}
	if( SensorValue[RightClawPot] < 2600 || vexRT[Btn6UXmtr2] )
	{

		motor[ RightClaw ] = rpwr;

	}
	else
	{

		motor[ RightClaw ] = 10;

	}

	if( vexRT[Btn8UXmtr2] )
 	{

 		doClawHold = true;

  	lClawTarget = 1500;
  	rClawTarget = 2450;

  }
  else
  {

  	doClawHold = false;

  }

  if( vexRT[Btn8DXmtr2] )
  {

  	doClawHold = true;

  	lClawTarget = 2500;
  	rClawTarget = 1500;

  }

}

void clawPos( int pos )
{

	switch( pos )
	{

		case 0:

			lClawTarget = 350;
			rClawTarget = 1050;

		break;

		case 1:

			lClawTarget = 1400;
			rClawTarget = 2450;

		break;

		default:



		break;

	}

}
