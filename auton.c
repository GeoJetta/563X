void auton1()
{

	//initialize
	rDriveTarget = 0;
	lDriveTarget = 0;
	liftTarget = 0;
	wait1Msec(750);

	//Drive backwards
	rDriveTarget = -461;
	lDriveTarget = -485;
	waitForPID();

	//Drive forwards to put down intake
	rDriveTarget = 104;
	lDriveTarget = 77;
	waitForPID();

	//Pick up stars
	rDriveTarget = 1100;
	lDriveTarget = 1100;
	waitForPID();

	//Lift up
	liftTarget = 607;
	waitForPID();

	//Back up to square
	rDriveTarget = 6;
	lDriveTarget = -18;
	waitForPID();

	//Point turn left
	rDriveTarget = 354;
	lDriveTarget = -440;
	waitForPID();

	//Back up to fence
	rDriveTarget = -1035;
	lDriveTarget = -1748;
	waitForPID();

	//Throw stars
	liftTarget = 1130;
	waitForPID();

	//Lift down
	liftTarget = -83;
	waitForPID();

	//Turn right to cube
	rDriveTarget = -1132;
	lDriveTarget = -1377;
	waitForPID();

	//Go to cube
	rDriveTarget = -499;
	lDriveTarget = -778;
	waitForPID();

	rDriveTarget = -5;
	lDriveTarget = -846;
	liftTarget = -83;
	waitForPID();

	rDriveTarget = 546;
	lDriveTarget = -281;
	liftTarget = -83;
	waitForPID();

	rDriveTarget = 384;
	lDriveTarget = -449;
	liftTarget = 287;
	waitForPID();

	rDriveTarget = 328;
	lDriveTarget = -454;
	liftTarget = 728;
	waitForPID();

	rDriveTarget = -597;
	lDriveTarget = -1379;
	liftTarget = 728;
	waitForPID();

	rDriveTarget = -559;
	lDriveTarget = -1377;
	liftTarget = 1135;
	waitForPID();

}

void rMiddleStarsCube()
{

	//initialize
	nMotorEncoder( L ) = 0;
	nMotorEncoder( R ) = 0;
	nMotorEncoder( R2 ) = 0;
	wait1Msec(200);

	//Drive backwards
	getToValue( ALL_DRIVE_MOTORS, -100, 60 );
	wait1Msec( 500 );

	//Drive forwards to put down intake
	getToValue( ALL_DRIVE_MOTORS, 100, 60 );
	wait1Msec( 500 );

	//Pick up stars
	getToValue( ALL_DRIVE_MOTORS, 300, 60 );

	//Lift up
	//getToValue( LIFT_MOTORS, 600 );

	while( SensorValue( R_LIFT_ENC ) > -460 )
	{

		lift(70);

	}

	lift(30);

	//Back up to square
	getToValue( ALL_DRIVE_MOTORS, -200, 60 );

	wait1Msec( 750 );

	//Point turn left
	getToValue( POINT_LEFT, 300, 40 );

	wait1Msec( 750 );

	//Back up to fence
	getToValue( ALL_DRIVE_MOTORS, -1000, 60 );

	//Throw stars
	//getToValue( LIFT_MOTORS, 1100 );

	//Lift down
	//getToValue( LIFT_MOTORS, 0 );

	while( SensorValue( R_LIFT_ENC ) > -1050 )
	{

		lift();
		getToValue( ALL_DRIVE_MOTORS, -300, 30 );

	}

	lift(0);

	wait1Msec( 500 );

	while( SensorValue( R_LIFT_ENC ) < -300 )
	{

		lift( -127 );

	}

	lift(0);

	wait1Msec( 500 );

	//Turn right to cube
	getToValue( POINT_RIGHT, 350, 40 );

	wait1Msec( 500 );

	//Go to cube
	getToValue( ALL_DRIVE_MOTORS, 500, 60 );

	while( SensorValue( R_LIFT_ENC ) > -500 )
	{

		lift();

	}

	lift(0);

	//getToValue( POINT_LEFT, 300, 40 );

	wait1Msec( 500 );

	getToValue( ALL_DRIVE_MOTORS, -500, 90 );

	while( SensorValue( R_LIFT_ENC ) > -1050 )
	{

		lift();

	}

	lift(0);

}

void lCornerStarFence()
{

	//initialize
	nMotorEncoder( L ) = 0;
	nMotorEncoder( R ) = 0;
	nMotorEncoder( R2 ) = 0;
	wait1Msec(200);

	//Drive backwards
	getToValue( ALL_DRIVE_MOTORS, -100, 60 );
	wait1Msec( 500 );

	//Drive forwards to put down intake and pick up star
	getToValue( ALL_DRIVE_MOTORS, 160, 60 );
	wait1Msec( 500 );

	while( SensorValue( R_LIFT_ENC ) > -460 )
	{

		lift(70);

	}

	lift(10);

	wait1Msec( 750 );

	//Point turn left
	getToValue( POINT_LEFT, 370, 40 );

	wait1Msec( 750 );

	//Back up to fence
	getToValue( ALL_DRIVE_MOTORS, -1000, 60 );

	while( SensorValue( R_LIFT_ENC ) > -900 )
	{

		lift();
		getToValue( ALL_DRIVE_MOTORS, -300, 30 );

	}

	lift(0);

	while( SensorValue( R_LIFT_ENC ) < -875 )
	{

		lift( -50 );

	}

	lift(5);

	getToValue( ALL_DRIVE_MOTORS, 200, 60 );

	wait1Msec( 500 );

	getToValue( POINT_LEFT, 450, 40 );

	wait1Msec( 500 );

	getToValue( ALL_DRIVE_MOTORS, 200, 60 );

	getToValue( POINT_LEFT, 400, 40 );

	wait1Msec( 500 );

	getToValue( ALL_DRIVE_MOTORS, 200, 60 );

	wait1Msec( 500 );

}

void rCornerStarFence()
{

	//initialize
	nMotorEncoder( L ) = 0;
	nMotorEncoder( R ) = 0;
	nMotorEncoder( R2 ) = 0;
	wait1Msec(200);

	//Drive backwards
	getToValue( ALL_DRIVE_MOTORS, -100, 60 );
	wait1Msec( 500 );

	//Drive forwards to put down intake and pick up star
	getToValue( ALL_DRIVE_MOTORS, 160, 60 );
	wait1Msec( 500 );

	while( SensorValue( R_LIFT_ENC ) > -460 )
	{

		lift(70);

	}

	lift(10);

	wait1Msec( 750 );

	//Point turn left
	getToValue( POINT_RIGHT, 480, 40 );

	wait1Msec( 750 );

	//Back up to fence
	getToValue( ALL_DRIVE_MOTORS, -1000, 60 );

	while( SensorValue( R_LIFT_ENC ) > -900 )
	{

		lift();
		getToValue( ALL_DRIVE_MOTORS, -300, 30 );

	}

	lift(0);

	while( SensorValue( R_LIFT_ENC ) < -860 )
	{

		lift( -50 );

	}

	lift(5);

	getToValue( ALL_DRIVE_MOTORS, 200, 60 );

	wait1Msec( 500 );

	getToValue( POINT_RIGHT, 450, 40 );

	wait1Msec( 500 );

	getToValue( ALL_DRIVE_MOTORS, 200, 60 );

	getToValue( POINT_RIGHT, 500, 40 );

	wait1Msec( 500 );

	getToValue( ALL_DRIVE_MOTORS, 200, 60 );

	wait1Msec( 500 );

}

void driveDist ( int dist, int pwr1, int pwr2 = pwr1 )
{

	resetMotorEncoder( L );
  resetMotorEncoder( R );

  while(abs(getMotorEncoder( R )) < abs(dist))
  {
    motor[ L ] = pwr1;
    motor[ R ] = pwr2;
  }

  motor[ R ] = 0;
  motor[ L ] = 0;

}

void turnAng ( int ang, int pwr1, int pwr2 = pwr1 )
{

	//resetMotorEncoder( L );
 // resetMotorEncoder( R );

	SensorValue(Gyro) = 0;

  while(abs(SensorValue(Gyro)) < abs(ang))
  {
    motor[ L ] = pwr1;
    motor[ R ] = pwr2;
  }

  motor[ R ] = 0;
  motor[ L ] = 0;

}


void liftDist ( int dist, int pwr )
{

	resetMotorEncoder( L2 );
  resetMotorEncoder( R2 );

  while(abs(getMotorEncoder( R2 )) < abs(dist))
  {
    lift( pwr );
  }

  haltLift();

}


void fenceLeft ( int wait = 300 )
{

	//at 50 power, 3450 = 360 degrees
	//at 70, 3150 = 360 degrees

	liftDist( 100, 127 ); //Lift a little
	wait1Msec( wait );
	driveDist( 550, 70 ); //Drive to fence
	wait1Msec( wait );
	liftDist( 530, 127 ); //Lift up
	wait1Msec( wait );
	driveDist( 470, 50 ); //Hit stars
	wait1Msec( wait );
	driveDist( -350, -50 ); //Back up
	wait1Msec( wait );
	//driveDist( 200, -50, 50 );
	turnAng( 280, 50, -50 ); //Turn right
	wait1Msec( wait );
	driveDist( 200, 50 ); //Hit stars
	wait1Msec( wait );
	turnAng( 530, -50, 50 ); //Turn Left
	wait1Msec( wait );
	driveDist( -400, -50 ); // Back up
	wait1Msec( wait );
	turnAng( 400, 50, -50 );
	wait1Msec( wait );
	liftDist( -50, -30 );
	driveDist( 460, 50 );
	wait1Msec( wait );

}

void fenceRight ( int wait = 300 )
{

	//at 50 power, 3450 = 360 degrees
	//at 70, 3150 = 360 degrees

	liftDist( 100, 127 );
	wait1Msec( wait );
	driveDist( 550, 70 );
	wait1Msec( wait );
	liftDist( 530, 127 );
	wait1Msec( wait );
	driveDist( 470, 50 );
	wait1Msec( wait );
	driveDist( -350, -50 );
	wait1Msec( wait );
	//driveDist( 200, -50, 50 );
	turnAng( 280, -50, 50 );
	wait1Msec( wait );
	driveDist( 200, 50 );
	wait1Msec( wait );
	turnAng( 350, 50, -50 );
	wait1Msec( wait );
	driveDist( -400, -50 );
	wait1Msec( wait );
	turnAng( 400, -50, 50 );
	wait1Msec( wait );
	liftDist( -50, -30 );
	driveDist( 460, 50 );
	wait1Msec( wait );

}

// DECLARE VARIABLES ETC

const short leftButton = 1;
const short centerButton = 2;
const short rightButton = 4;

//LCD FUNCTIONS

string mainBattery, backupBattery; //Strings to hold battery values

void LCDInit ()
{

	bLCDBacklight = true;

}

void LCDClear ()
{

	clearLCDLine( 0 );                                            // Clear line 1 (0) of the LCD
	clearLCDLine( 1 );                                            // Clear line 2 (1) of the LCD

}

void waitForPress ()
{

	while(nLCDButtons == 0){}
	wait1Msec(5);

}

void waitForRelease ()
{

	while(nLCDButtons != 0){}
	wait1Msec(5);

}

int choice = 0;

void chooseAuton ()
{

	//Declare count variable to keep track of our choice
	int count = 0;

	//------------- Beginning of User Interface Code ---------------

	//Clear LCD
	LCDClear();

	//Loop while center button is not pressed
	while(nLCDButtons != centerButton)
	{

		//Switch case that allows the user to choose from 4 different options
		switch( count )
		{

			case 0:

				//Display first choice
				//displayLCDCenteredString(0, "Middle Stars, Cube (R)");
				//displayLCDCenteredString(1, "<         Enter        >");
				displayLCDString( 0, 0, "R Mid Stars+Cube" );
				displayLCDString( 1, 0, "<    Enter     >" );
				waitForPress();

				//Increment or decrement "count" based on button press
				if(nLCDButtons == leftButton)
				{

					waitForRelease();
					count = 4;

				}

				else if(nLCDButtons == rightButton)
				{

					waitForRelease();
					count++;

				}

			break;

			case 1:

				//Display second choice
				//displayLCDCenteredString(0, "Right Corner Star, Fence");
				//displayLCDCenteredString(1, "<         Enter        >");
				displayLCDString( 0, 0, "   Left Fence   " );
				displayLCDString( 1, 0, "<    Enter     >" );
				waitForPress();

				//Increment or decrement "count" based on button press
				if(nLCDButtons == leftButton)
				{

					waitForRelease();
					count--;

				}

				else if(nLCDButtons == rightButton)
				{

					waitForRelease();
					count++;

				}

			break;

			case 2:

				//Display second choice
				//displayLCDCenteredString(0, "Right Corner Star, Fence");
				//displayLCDCenteredString(1, "<         Enter        >");
				displayLCDString( 0, 0, "   Right Fence  " );
				displayLCDString( 1, 0, "<    Enter     >" );
				waitForPress();

				//Increment or decrement "count" based on button press
				if(nLCDButtons == leftButton)
				{

					waitForRelease();
					count--;

				}

				else if(nLCDButtons == rightButton)
				{

					waitForRelease();
					count++;

				}

			break;

			case 3:

				//Display second choice
				//displayLCDCenteredString(0, "Right Corner Star, Fence");
				//displayLCDCenteredString(1, "<         Enter        >");
				displayLCDString( 0, 0, "R Corner Str+Fnc" );
				displayLCDString( 1, 0, "<    Enter     >" );
				waitForPress();

				//Increment or decrement "count" based on button press
				if(nLCDButtons == leftButton)
				{

					waitForRelease();
					count--;

				}

				else if(nLCDButtons == rightButton)
				{

					waitForRelease();
					count++;

				}

			break;

			case 4:

				//Display third choice
				//displayLCDCenteredString(0, "Left Corner Star, Fence");
				//displayLCDCenteredString(1, "<         Enter        >");
				displayLCDString( 0, 0, "L Corner Str+Fnc" );
				displayLCDString( 1, 0, "<    Enter     >" );
				waitForPress();

				//Increment or decrement "count" based on button press
				if(nLCDButtons == leftButton)
				{

					waitForRelease();
					count--;

				}

				else if(nLCDButtons == rightButton)
				{

					waitForRelease();
					count = 0;

				}

			break;

			default:

				count = 0;

			break;

		}

		wait1Msec(20);

	}

	//return count;

	choice = count;

}

void runLCDAuton ()
{

	switch( choice )
	{

		case 0:

			rMiddleStarsCube();

		break;

		case 1:

			fenceLeft();

		break;

		case 2:

			fenceRight();

		break;

		case 3:

			rCornerStarFence();

		break;

		case 4:

			lCornerStarFence();

		break;

	}

}


void primaryBatteryDisplay ( int line = 0 )
{

	//Display the Primary Robot battery voltage
	displayLCDString( line, 0, "Primary: " );
	sprintf( mainBattery, "%1.2f%c", nImmediateBatteryLevel / 1000.0,'V' ); //Build the value to be displayed
	displayNextLCDString( mainBattery );

}

void backupBatteryDisplay ( int line = 1 )
{

	//Display the Backup battery voltage
	displayLCDString( line, 0, "Backup: " );
	sprintf( backupBattery, "%1.2f%c", BackupBatteryLevel / 1000.0, 'V' );    //Build the value to be displayed
	displayNextLCDString( backupBattery );

}
