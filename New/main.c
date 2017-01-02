#define NUMBER_OF_MOTORS kNumbOfTotalMotors

int lDriveTarget;
int rDriveTarget;

enum motorGroup_enum
{

	ALL_MOTORS 					= 0,
	LEFT_DRIVE_MOTORS 	= 1,
	RIGHT_DRIVE_MOTORS 	= 2,
	LIFT_MOTORS 				= 3

};

tMotor lift_motor_group
{

	//insert lift motor names

};

tMotor left_drive_motor_group
{

	//insert drive motor names

};

tMotor right_drive_motor_group
{

	//insert drive motor names

};

//PID values for all of drive
struct driveValues
{

	float kP = 0;
	float kI = 0;
	float kD = 0;
	float current;
	float target;
	float error;
	float integral;
	float derivative;
	float lastError;
	float final;

} drivePID;


//PID values for right side of drive
struct rDriveValues
{

	float kP = 0;
	float kI = 0;
	float kD = 0;
	float current;
	float target;
	float error;
	float integral;
	float derivative;
	float lastError;
	float final;

} rDrivePID;

//PID values for left side of drive
struct lDriveValues
{

	float kP = 0;
	float kI = 0;
	float kD = 0;
	float current;
	float target;
	float error;
	float integral;
	float derivative;
	float lastError;
	float final;

} lDrivePID;

struct liftValues
{

	float kP = 0;
	float kI = 0;
	float kD = 0;
	float current;
	float target;
	float error;
	float integral;
	float derivative;
	float lastError;

} liftPID;

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
			for( int i = 0; i < (NUMBER_OF_MOTORS-1); i++ )
			{

				motor[i] = pwr;

			}

		break;

		//left drive
		case 1:

			//set power to each motor on the left of the drive
			for( int i = 0; i < sizeOf(left_drive_motor_group)/sizeOf(int); i++ )
			{

				motor[left_drive_motor_group[i]] = pwr;

			}

		break;

		//right drive
		case 2:

			//set power to each motor on the left of the drive
			for( int i = 0; i < sizeOf(right_drive_motor_group)/sizeOf(int); i++ )
			{

				motor[right_drive_motor_group[i]] = pwr;

			}

		break;

		//lift
		case 3:

			//set power to each motor on the lift
			for( int i = 0; i < sizeOf(lift_drive_motor_group)/sizeOf(int); i++ )
			{

				motor[lift_drive_motor_group[i]] = pwr;

			}

		break;

		default:

		break;

	}

}

//PID control for drive
task lDrivePIDControl(  )
{

	//Pass target value
	lDrivePID.target 		= lDriveTarget;
	//Get current position from sensors
	lDrivePID.current		= sensorValue( L_DRIVE_ENC );
	//Calculate how far away from target
	lDrivePID.error			= lDrivePID.target - lDrivePID.current;

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

	//Calculate Derivative (difference between current and last error)
	lDrivePID.derivative	= lDrivePID.error - lDrivePID.lastError;
	//Pass final setting
	lDrivePID.final			= ( lDrivePID.error * lDrivePID.kP ) + ( lDrivePID.integral * lDrivePID.kI ) + ( lDrivePID.derivative * lDrivePID.kD );

	//Apply calculated PID to motors
	setMotors( LEFT_DRIVE_MOTORS, lDrivePID.final );

}

//PID control for drive
task rDrivePIDControl( int target )
{

	//Pass target value
	rDrivePID.target 		= rDriveTarget;
	//Get current position from sensors
	rDrivePID.current		= sensorValue( R_DRIVE_ENC );
	//Calculate how far away from target
	rDrivePID.error			= rDrivePID.target - rDrivePID.current;

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

	//Calculate Derivative (difference between current and last error)
	rDrivePID.derivative	= rDrivePID.error - rDrivePID.lastError;
	//Pass final setting
	rDrivePID.final			= ( rDrivePID.error * rDrivePID.kP ) + ( rDrivePID.integral * rDrivePID.kI ) + ( rDrivePID.derivative * rDrivePID.kD );

	//Apply calculated PID to motors
	setMotors( RIGHT_DRIVE_MOTORS, rDrivePID.final );

}
