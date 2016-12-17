/*--- TEMP MOTOR PORTS ---*/

/*--- L1: Port 1 -------*/
/*--- R1: Port 2 -------*/
/*--- L2: Port 3 -------*/
/*--- R2: Port 4 -------*/

/*--- RF: A - Port 5 ---*/
/*--- LF: B - Port 5 ---*/
/*--- LB: C - Port 6 ---*/
/*--- RB: D - Port 6 ---*/

/*--- L3: Port 7 -------*/
/*--- R3: Port 8 -------*/
/*--- L4: Port 9 -------*/
/*--- R4: Port 10 ------*/



/*--- ROBOT INFORMATION ---------*/



/*--------- MOTORS --------------*/


/*---------- LIFT ---------------*/

/*--- L/R: Left/Right -----------*/
/*--- #: Starting from top ------*/
/*--- EXAMPLE: L1 = Left-Top ----*/

/*--------- DRIVE ---------------*/

/*--- L/R: Left/Right -----------*/
/*--- B/F: Back/Front -----------*/
/*--- EXAMPLE: RF = Left-Back ---*/


// DECLARE VARIABLES ETC

int old_ds_dist[3]; //Create a place to store original encoder values to subtract

const short leftButton = 1;
const short centerButton = 2;
const short rightButton = 4;


//DRIVER CONTROL - BASE FUNCTIONS

const	int tsArray[128] = {
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
		88, 89, 89, 90, 90,127,127,127
};

int trueSpeed(int power){
	return ((power>0)?1:-1)*tsArray[power*((power>0)?1:-1)];
}

void drive ( int l = 127, int r = l, int sped = true ) //Default power max forward, default left and right drive equal to this power
{

	motor [ L ] = l; //Set left drive equal to left power

	motor [ R ] = r;

}

void lift ( int l = 127, int r = l )
{

	motor [ L1 ] = l; //top
	motor [ L2 ] = l;
	motor [ L3 ] = l;
	motor [ L4 ] = l; //bottom

	motor [ R4 ] = r; //bottom
	motor [ R3 ] = r;
	motor [ R2 ] = r;
	motor [ R1 ] = r; //top

}

void reset_ds_dist () //Allow user to reset starting values whenever needed
{

	old_ds_dist[1] = sensorValue[LEFTENC]; //Set array to current encoder values to subtract from
	old_ds_dist[2] = sensorValue[RIGHTENC];

}



//AUTONOMOUS - COMPLEX FUNCTIONS

void liftToAngle ( int ang, int pwr = 127, int low_pwr = 10 )
{

	int angle = ang;

	if ( ang > 0 )
	{

		if ( ( abs( SensorValue( LLIFTENC ) ) + abs( SensorValue( RLIFTENC ) ) ) / 2 / 5 > angle )
		{

			while ( ( abs( SensorValue( LLIFTENC ) ) + abs( SensorValue( RLIFTENC ) ) ) / 2 / 5 > angle )
			{

				lift( -pwr*0.2, -pwr*0.2 );

			}

			lift( pwr );

		}
		else
		{

			while ( ( abs( SensorValue( LLIFTENC ) ) + abs( SensorValue( RLIFTENC ) ) ) / 2 / 5 < angle )
			{

				lift( pwr );

			}

			lift( pwr );

		}

	}

}

void point ( int pwr, bool dir, int err = 5, int mult = 0.1 ) //Point turn keeping opposite encoder values. No ramp up, just checking if each side is opposite
{

	int l; //left sign value
	int r; //right sign value

	if ( dir == LEFT ) //Check if strings need quotes.
	{

		l = -1; //multipliers for drive values
		r = 1;

	}

	else if ( dir == RIGHT )
	{

		l = 1;
		r = -1;

	}

	drive( pwr * l, pwr * r );

	if ( abs( sensorValue[LEFTENC] ) - old_ds_dist[1] - err < abs( sensorValue[RIGHTENC] ) - old_ds_dist[2] + err && abs( sensorValue[LEFTENC] ) - old_ds_dist[1] + err > abs( sensorValue[RIGHTENC] ) - old_ds_dist[2] - err ) //If they are turning an equal amount within margin of error, set power to the same
	{

		drive( pwr * l, pwr * r );

	}

	else if ( abs( sensorValue[LEFTENC] ) - old_ds_dist[1] - err < abs( sensorValue[RIGHTENC] ) - old_ds_dist[2] + err )
	{

		drive( pwr * l, pwr * ( 1 - mult ) * r );

	}

	else if ( abs( sensorValue[LEFTENC] ) - old_ds_dist[1] + err > abs( sensorValue[RIGHTENC] ) - old_ds_dist[2] - err )
	{

		drive( pwr * ( 1 - mult ) * l, pwr * r );

	}


}

//NOTE: if negative encoder values start messing stuff up, use absolute values in ds

void ds ( int pwr, int err = 5, int mult = 0.1 ) //Drive straight. No ramp up, just checking each side is equal
{

	drive( pwr, pwr );

	if ( abs( sensorValue[LEFTENC] ) - old_ds_dist[1] - err < abs( sensorValue[RIGHTENC] ) - old_ds_dist[2] + err && abs( sensorValue[LEFTENC] ) - old_ds_dist[1] + err > abs( sensorValue[RIGHTENC] ) - old_ds_dist[2] - err ) //If they are straight within margin of error, set power to the same
	{

		drive( pwr, pwr );

	}

	else if ( abs( sensorValue[LEFTENC] ) - old_ds_dist[1] - err < abs( sensorValue[RIGHTENC] ) - old_ds_dist[2] + err )
	{

		drive( pwr, pwr * ( 1 - mult ) );

	}

	else if ( abs( sensorValue[LEFTENC] ) - old_ds_dist[1] + err > abs( sensorValue[RIGHTENC] ) - old_ds_dist[2] - err )
	{

		drive( pwr * ( 1 - mult ), pwr );

	}

}

void ds_dist ( int dist, int pwr = 127, int ramp = 5, int slow_dist = 12 ) //Drive straight for a certain distance (Includes ramp-up)
{

	for (int i; i < pwr; i++) //Ramp up power smoothly
	{

		ds( i );

		wait1MSec( ramp );

	}

	while ( ( ( abs( sensorValue[LEFTENC] ) - old_ds_dist[1] ) + ( abs( sensorValue[RIGHTENC] ) - old_ds_dist[2] ) ) / 180 < dist - slow_dist ) //Go at pwr until within slow_dist of target
	{

		ds( pwr );

	}

	while ( ( ( abs( sensorValue[LEFTENC] ) - old_ds_dist[1] ) + ( abs( sensorValue[RIGHTENC] ) - old_ds_dist[2] ) ) / 180 < dist )
	{

		//ds( pwr * -( ( ( ( abs( sensorValue[LEFTENC] ) - old_ds_dist[1] ) + ( abs( sensorValue[RIGHTENC] ) - old_ds_dist[2] ) ) / 180 ) - dist ) / 20 );

		ds( pwr * 0.3 );

	}

	drive( 0, 0 );

}

void point_dist ( bool dir, int dist, int pwr = 127, int ramp = 5, int slow_dist = 12 )
{

	for (int i; i < pwr; i++) //Ramp up power smoothly
	{

		point( i, dir );

		wait1MSec( ramp );

	}

	while ( ( ( abs( sensorValue[LEFTENC] ) - old_ds_dist[1] ) + ( abs( sensorValue[RIGHTENC] ) - old_ds_dist[2] ) ) / 180 < dist - slow_dist ) //Go at pwr until within slow_dist of target
	{

		point( pwr, dir );

	}

	while ( ( ( abs( sensorValue[LEFTENC] ) - old_ds_dist[1] ) + ( abs( sensorValue[RIGHTENC] ) - old_ds_dist[2] ) ) / 180 < dist )
	{

		ds( pwr * -( ( ( ( abs( sensorValue[LEFTENC] ) - old_ds_dist[1] ) + ( abs( sensorValue[RIGHTENC] ) - old_ds_dist[2] ) ) / 180 ) - dist ) / 20, dir );

	}

	drive( 0, 0 );

}

//void point_ang ( int ang, int pwr = 127, int ramp = 5, int slow_angle = 20 ) //point turn to a specific angle
//{

//	string dir;

//	if ( GyroGetAngle() - ang >= 0 ) //if current angle - wanted angle is greater than 0, turn right
//	{

//		dir = "right";

//	}

//	else if ( GyroGetAngle() - ang < 0 ) //if current angle - wanted angle is greater than 0, turn right
//	{

//		dir = "left";

//	}

//	for (int i; i < pwr; i++) //Ramp up power smoothly
//	{

//		point( i, dir );

//		wait1MSec( ramp );

//	}

//	while ( /*abs(*/ ( GyroGetAngle() - ang ) /*)*/ > 0 + slow_angle ) //Go at pwr until within slow_angle of target
//	{

//		point( pwr, dir );

//	}

//	while ( /*abs*/( GyroGetAngle() - ang /*)*/ >= 0 )
//	{

//		point( pwr * /*abs(*/ GyroGetAngle() - ang /*)*/ / 20, dir );

//	}

//	drive( 0, 0 );

//}

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

void haltLift ()
{

	motor[L1] = 5;
	motor[L2] = 5;
	motor[L3] = 5;
	motor[L4] = 5;

	motor[R4] = 5;
	motor[R3] = 5;
	motor[R2] = 5;
	motor[R1] = 5;

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

//VARIOUS AUTONS

/*
void setPIDAll ( bool set = true )
{

	setPIDforMotor(L, set);
	setPIDforMotor(R, set);
	setPIDforMotor(L2, set);
	setPIDforMotor(R2, set);

}
*/

void pidTest()
{

	//setPIDAll( true );

	//setMotorTarget(L, 500, 110, true);
	//setMotorTarget(R, 500, 110, true);

	//setPIDAll( false );

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

void cubeRight ()
{

	driveDist( 500, 70 );
	wait1Msec( 300 );
	driveDist( 300, 100 );
	wait1Msec( 500 );
	liftDist( 500, 127 );
	driveDist( 250, -80, 80);
	wait1Msec( 500 );
	driveDist( -400, -100 );
	wait1Msec( 500 );
	liftDist( 450, 127 );

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

void cubeLeft ()
{

	driveDist( 500, 70 );
	wait1Msec( 300 );
	driveDist( 300, 100 );
	wait1Msec( 500 );
	liftDist( 500, 127 );
	driveDist( 250, 80, -80);
	wait1Msec( 500 );
	driveDist( -400, -100 );
	wait1Msec( 500 );
	liftDist( 450, 127 );

}


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
				displayLCDCenteredString(0, "Left Fence");
				displayLCDCenteredString(1, "<         Enter        >");
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
				displayLCDCenteredString(0, "Right Cube");
				displayLCDCenteredString(1, "<         Enter        >");
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

				//Display third choice
				displayLCDCenteredString(0, "Right Fence");
				displayLCDCenteredString(1, "<         Enter        >");
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

				//Display fourth choice
				displayLCDCenteredString(0, "Left Cube");
				displayLCDCenteredString(1, "<         Enter        >");
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
				displayLCDCenteredString(0, "pidTest");
				displayLCDCenteredString(1, "<         Enter        >");
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

	}

	//return count;

	choice = count;

}

void runLCDAuton ()
{

	switch( choice )
	{

		case 0:

			fenceLeft();

		break;

		case 1:

			cubeRight();

		break;

		case 2:

			fenceRight(250);

		break;

		case 3:

			fenceLeft();

		break;

		case 4:

			pidTest();

		break;

	}

}

string x_coord, y_coord;

void GPSDisplay ( )
{

	//Display the x coordinate
	displayLCDString( 0, 0, "X: " );
	sprintf( x_coord, "%d", old_x ); //Build the value to be displayed
	displayNextLCDString( x_coord );

	//Display the y coordinate
	displayLCDString( 1, 0, "Y: " );
	sprintf( y_coord, "%d", old_y ); //Build the value to be displayed
	displayNextLCDString( y_coord );

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


//MISC FUNCTIONS


//TEST ALL LIFT MOTORS INDIVIDUALLY (IN PAIRS)

void motTest ()
{

	motor[L1] = 127;
	motor[R1] = 127;

	wait1Msec(2000);

	lift(0);

	motor[L2] = 127;
	motor[R2] = 127;

	wait1Msec(2000);

	lift(0);

	motor[L3] = 127;
	motor[R3] = 127;

	wait1Msec(2000);

	lift(0);

	motor[L4] = 127;
	motor[R4] = 127;

	wait1Msec(2000);

	lift(0);

}
