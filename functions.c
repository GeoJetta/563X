#define NUMBER_OF_MOTORS kNumbOfTotalMotors

int lDriveTarget;
int rDriveTarget;
int liftTarget;

enum motorGroup_enum
{

	ALL_MOTORS 					= 0,
	LEFT_DRIVE_MOTORS 	= 1,
	RIGHT_DRIVE_MOTORS 	= 2,
	LIFT_MOTORS 				= 3,
	ALL_DRIVE_MOTORS		= 4

};

tMotor lift_motor_group[8] =
{

	L1,
	R1,
	L2,
	R2,
	L3,
	R3,
	L4,
	R4

};

tMotor left_drive_motor_group[1] =
{

	L

};

tMotor right_drive_motor_group[1] =
{

	R

};

//PID values for right side of drive
typedef struct rDriveValues
{

	float kP;
	float kI;
	float kD;
	float current;
	float target;
	float error;
	float integral;
	float derivative;
	float lastError;
	float final;
	int threshold;

} rDriveTheoryPID;

//PID values for left side of drive
typedef struct lDriveValues
{

	float kP;
	float kI;
	float kD;
	float current;
	float target;
	float error;
	float integral;
	float derivative;
	float lastError;
	float final;
	int threshold;

} lDriveTheoryPID;

typedef struct liftValues
{

	float kP;
	float kI;
	float kD;
	float current;
	float target;
	float error;
	float integral;
	float derivative;
	float lastError;
	float final;
	int threshold;

} liftTheoryPID;

void clipValues( int input, int high = 128, int low = -128 )
{

	if ( input >= high )
		input = high;
	else if ( input <= low )
		input = low;

}

//function to set motor power to robot subsystems
void setMotors( int motorGroup, int pwr )
{

	//clip values for motors
	clipValues( pwr );

	//decide where to send power
	switch( motorGroup )
	{

		//All motors
		case 0:

			//set power for each motor. -1 because arrays start at 0
			for( unsigned int i = 0; i < NUMBER_OF_MOTORS; i++ )
			{

				motor[i] = pwr;

			}

		break;

		//left drive
		case 1:

			//set power to each motor on the left of the drive
			for( int i = 0; i < sizeof(left_drive_motor_group); i++ )
			{

				motor[left_drive_motor_group[i]] = pwr;

			}

		break;

		//right drive
		case 2:

			//set power to each motor on the right of the drive
			for( int i = 0; i < sizeof(right_drive_motor_group); i++ )
			{

				motor[right_drive_motor_group[i]] = pwr;

			}

		break;

		//lift
		case 3:

			//set power to each motor on the lift
			for( int i = 0; i < sizeof(lift_motor_group); i++ )
			{

				motor[lift_motor_group[i]] = pwr;

			}

		break;

		//both sides of drive
		case 4:

			//set power to each motor on right of the drive
			for( int i = 0; i < sizeof(right_drive_motor_group); i++ )
			{

				motor[right_drive_motor_group[i]] = pwr;
				motor[left_drive_motor_group[i]] = pwr;

			}

		break;

		default:

		break;

	}

}

//Put theoretical structs into ones we can use in our functions
lDriveTheoryPID lDrivePID;
rDriveTheoryPID rDrivePID;
liftTheoryPID liftPID;

//bools to check if each loop has reached its target
bool lDrivePIDTargetCompleted = false;
bool rDrivePIDTargetCompleted = false;
bool liftPIDTargetCompleted = false;

//int CheckPID[3] = [0,0,0];

//PID control for left side of drive
task lDrivePIDControl(  )
{

	lDrivePID.kP = 1.9;
	lDrivePID.kD = 2.6;
	lDrivePID.kI = 0.01;
	lDrivePID.threshold = 3;

	while(1)
	{

		//Pass target value
		lDrivePID.target 		= lDriveTarget;
		//Get current position from sensors
		lDrivePID.current		= SensorValue( L_DRIVE_ENC );
		//Calculate how far away from target
		lDrivePID.error			= lDrivePID.target - lDrivePID.current;

		if( abs( lDrivePID.error ) > lDrivePID.threshold)
		{

			// integral - if Ki is not 0
		  if( lDrivePID.kI != 0 )
		  {
		  	// If we are inside controlable window then integrate the error
		    if( abs( lDrivePID.error ) < PID_INTEGRAL_LIMIT )
		      lDrivePID.integral += lDrivePID.error;
		    else
		      lDrivePID.integral = 0;
		  }
		  else
		    lDrivePID.integral = 0;

		  }

		//Calculate Derivative (difference between current and last error)
		lDrivePID.derivative	= lDrivePID.error - lDrivePID.lastError;

		//Check if outside of threshold
		if( abs( lDrivePID.error ) > lDrivePID.threshold)
		{
			//Pass final setting
			lDrivePID.final			= ( lDrivePID.error * lDrivePID.kP ) + ( lDrivePID.integral * lDrivePID.kI ) + ( lDrivePID.derivative * lDrivePID.kD );

			lDrivePIDTargetCompleted = false;

		}
		else
		{
			//If within threshold, set motors to 0
			lDrivePID.final			= 0;

			lDrivePIDTargetCompleted = true;

		}

		//Apply calculated PID to motors
		setMotors( LEFT_DRIVE_MOTORS, lDrivePID.final );

		lDrivePID.lastError = lDrivePID.error;

		wait1Msec(20);

	}

}

//PID control for right side of drive
task rDrivePIDControl( )
{

	//rDrivePID.kP = 50;
	//rDrivePID.kD = -49;
	//rDrivePID.kI = 0.05;
	//rDrivePID.threshold = 3;

	rDrivePID.kP = lDrivePID.kP;
	rDrivePID.kD = lDrivePID.kD;
	rDrivePID.kI = lDrivePID.kI;
	rDrivePID.threshold = lDrivePID.threshold;

	while(1)
	{

		//Pass target value
		rDrivePID.target 		= rDriveTarget;
		//Get current position from sensors
		rDrivePID.current		= -SensorValue( R_DRIVE_ENC );
		//Calculate how far away from target
		rDrivePID.error			= rDrivePID.target - rDrivePID.current;

		if( abs( rDrivePID.error ) > rDrivePID.threshold)
		{

			// integral - if Ki is not 0
		  if( rDrivePID.kI != 0 )
		  {
		  	// If we are inside controlable window then integrate the error
		    if( abs( rDrivePID.error ) < PID_INTEGRAL_LIMIT )
		      rDrivePID.integral += rDrivePID.error;
		    else
		      rDrivePID.integral = 0;
		  }
		  else
		    rDrivePID.integral = 0;

		}

		//Calculate Derivative (difference between current and last error)
		rDrivePID.derivative	= rDrivePID.error - rDrivePID.lastError;

		//Check if outside of threshold
		if( abs( rDrivePID.error ) > rDrivePID.threshold)
		{
			//Pass final setting
			rDrivePID.final			= ( rDrivePID.error * rDrivePID.kP ) + ( rDrivePID.integral * rDrivePID.kI ) + ( rDrivePID.derivative * rDrivePID.kD );

			rDrivePIDTargetCompleted = false;

		}
		else
		{
			//If within threshold, set motors to 0
			rDrivePID.final			= 0;

			rDrivePIDTargetCompleted = true;

		}

		//Apply calculated PID to motors
		setMotors( RIGHT_DRIVE_MOTORS, rDrivePID.final );

		rDrivePID.lastError = rDrivePID.error;

		wait1Msec(20);

	}

}

//PID control for lift
task liftPIDControl( )
{

	liftPID.kP = 3;
	liftPID.kD = 14;
	liftPID.kI = 0.27;
	liftPID.threshold = 7;

	while(1)
	{

		//Pass target value
		liftPID.target 		= liftTarget;
		//Get current position from sensors
		liftPID.current		= -SensorValue( R_LIFT_ENC );
		//Calculate how far away from target
		liftPID.error			= liftPID.target - liftPID.current;

		//Check if outside of threshold
		if( abs( liftPID.error ) > liftPID.threshold)
		{

			// integral - if Ki is not 0
		  if( liftPID.kI != 0 )
		  {
		  	// If we are inside controlable window then integrate the error
		    if( abs( liftPID.error ) < PID_INTEGRAL_LIMIT )
		      liftPID.integral = liftPID.integral + liftPID.error;
		    else
		      liftPID.integral = 0;
		  }
		  else
		    liftPID.integral = 0;

		 }

		//Calculate Derivative (difference between current and last error)
		liftPID.derivative	= liftPID.error - liftPID.lastError;

		//Check if outside of threshold
		if( abs( liftPID.error ) > liftPID.threshold)
		{
			//Pass final setting
			liftPID.final			= ( liftPID.error * liftPID.kP ) + ( liftPID.integral * liftPID.kI ) + ( liftPID.derivative * liftPID.kD );

			liftPIDTargetCompleted = false;

		}
		else
		{
			//If within threshold, set motors to 0
			liftPID.final			= 0;

			liftPIDTargetCompleted = true;

		}

		//Apply calculated PID to motors
		setMotors( LIFT_MOTORS, liftPID.final );

		liftPID.lastError = liftPID.error;

		wait1Msec(20);

	}

}

bool recording = false;

task record( )
{

	while( 1 )
	{

		if (vexRT(Btn7L))
  	{

  		recording = true;

  		wait1Msec(50);

  	}

  	if (vexRT(Btn7R))
  	{

  		recording = false;

  		wait1Msec(50);

  	}

		while( recording )
		{

			//writeDebugStreamLine("rDriveTarget = %d;", -SensorValue(R_DRIVE_ENC));
			//writeDebugStreamLine("lDriveTarget = %d;", SensorValue(L_DRIVE_ENC));
			//writeDebugStreamLine("liftTarget = %d;", -SensorValue(R_LIFT_ENC));
			//writeDebugStreamLine("wait1Msec( REPLAY_DELAY );");

			//wait1Msec(RECORD_DELAY);

			if (vexRT(Btn8R))
  		{

  			writeDebugStreamLine("rDriveTarget = %d;", -SensorValue(R_DRIVE_ENC));
				writeDebugStreamLine("lDriveTarget = %d;", SensorValue(L_DRIVE_ENC));
				writeDebugStreamLine("liftTarget = %d;", -SensorValue(R_LIFT_ENC));

  			wait1Msec(200);

  		}

			if (vexRT(Btn7R))
  		{

  			recording = false;

  			wait1Msec(50);

  		}

		}

	}

}

bool waitForPID()
{

	while( !( lDrivePIDTargetCompleted && rDrivePIDTargetCompleted && liftPIDTargetCompleted ) )
	{

		wait1Msec( 20 );

	}

	wait1Msec( 750 );

	return true;

}

//task allPIDLoopsCompleted()
//{

//	while( true )
//	{

//		checkPIDLoops();

//		wait1Msec(20);

//	}

//}

//void waitForPID()
//{

//	while( checkPIDLoops() == false )
//	{

//		wait1Msec( 20 );

//	}

//}

void getToValue( int motorGroup, int value, int pwr = 127, int overshoot = 20 )
{

	int firstValue = 0;

	if( value < 0 )
	{

		pwr = -pwr;

	}

	switch ( motorGroup )
	{

		case LEFT_DRIVE_MOTORS:

			firstValue = SensorValue( L_DRIVE_ENC );

			while( fabs( ( SensorValue( L_DRIVE_ENC ) - firstValue ) ) < abs( ( value - overshoot ) ) )
			{

				setMotors( LEFT_DRIVE_MOTORS, pwr );

			}

			setMotors( LEFT_DRIVE_MOTORS, 0 );

		break;

		case RIGHT_DRIVE_MOTORS:

			firstValue = SensorValue( R_DRIVE_ENC );

			while( fabs( ( SensorValue( R_DRIVE_ENC )  - firstValue ) ) < abs( ( value - overshoot ) ) )
			{

				setMotors( RIGHT_DRIVE_MOTORS, pwr );

			}

			setMotors( RIGHT_DRIVE_MOTORS, 0 );

		break;

		case ALL_DRIVE_MOTORS:

			firstValue = ( SensorValue( L_DRIVE_ENC ) + SensorValue( R_DRIVE_ENC ) / 2 );

			while( fabs( ( ( SensorValue( L_DRIVE_ENC ) + SensorValue( R_DRIVE_ENC ) / 2 )  - firstValue ) ) < abs( ( value - overshoot ) ) )
			{

				setMotors( ALL_DRIVE_MOTORS, pwr );

			}

			setMotors( ALL_DRIVE_MOTORS, 0 );

		break;

		//case LIFT_MOTORS:

		//	//firstValue = abs( SensorValue( R2 ) );

		//	while( fabs( SensorValue(R2) ) <= value )
		//	{

		//		setMotors( LIFT_MOTORS, 50 );

		//	}

		//	setMotors( LIFT_MOTORS, 0);

		//break;

		case POINT_LEFT:

			firstValue = SensorValue( R_DRIVE_ENC );

			while( fabs( SensorValue( R_DRIVE_ENC ) - firstValue ) < ( value - overshoot ) )
			{

				setMotors( LEFT_DRIVE_MOTORS, -pwr );
				setMotors( RIGHT_DRIVE_MOTORS, pwr );

			}

			setMotors( ALL_DRIVE_MOTORS, 0 );

		break;

		case POINT_RIGHT:

			firstValue = SensorValue( L_DRIVE_ENC );

			while( fabs( SensorValue( L_DRIVE_ENC ) - firstValue ) < abs( ( value - overshoot ) ) )
			{

				setMotors( LEFT_DRIVE_MOTORS, pwr );
				setMotors( RIGHT_DRIVE_MOTORS, -( pwr ) );

			}

			setMotors( ALL_DRIVE_MOTORS, 0 );

		break;

	}

}
